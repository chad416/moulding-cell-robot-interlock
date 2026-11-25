"""Scenario runs for the PLC test bench (plc/test), one PLC scan per tick.

Each case runs the cell model of cell/system.py (machine model, TP program through the interpreter, compiled
PLC block) under a fixed schedule and records, for every PLC scan, the input image of CELL_PLC and the output
image it produced. The bench in plc/test replays the inputs to the block compiled by another compiler and
compares its outputs scan by scan.

The schedule is in scans, not in seconds: the bench has to finish in a few thousand scans. The durations below
are bench values, not machine times. A duration of None means "never happens".
"""

from __future__ import annotations

from typing import Callable, NamedTuple

from .system import System

DUR = {
    "imm:process_done": 45, "imm:open_start": 0, "imm:open_done": 4, "imm:ej_fwd_done": 2, "imm:ej_back_done": 2,
    "imm:close_start": 0, "imm:close_done": 5, "cell:part_gripped": 2, "cell:part_released": 2,
    "cell:cutter_closed": 2, "cell:cutter_open": 2, "cell:conveyor_indexed": 3,
    "rob:move_done": 3, "rob:delay": 2, "rob:timeout": 40,
}
ROBOT_NOW = ("rob:instr", "rob:wait_ok", "rob:move_start", "rob:alarm")
WATCHDOG_SCANS = 25
SETTLE = 6                      # scans recorded after the end condition


class Ctx:
    def __init__(self, sysm: System):
        self.sysm, self.s, self.scan = sysm, sysm.initial(), 0
        self.closes = 0
        self.memo: dict = {}

    def plc_state(self) -> str:
        return self.sysm.plc.name_of("S_", self.s.out[self.sysm.oi["STATE"]])

    def robot_waits_for_mould(self) -> bool:
        r = self.s.rob
        return r.target is None and r.pos == "WAIT_IMM" and "MOULD_OPEN]=ON OR" in self.sysm.prog.where(r)

    def after(self, key: str, scans: int) -> bool:
        """True once `scans` scans have passed since the first call with this key."""
        t0 = self.memo.setdefault(key, self.scan)
        return self.scan - t0 >= scans


class Case(NamedTuple):
    key: str
    title: str
    scenario: str = "none"
    dur: dict = {}                                                  # overrides of DUR
    inject: Callable[[Ctx, dict], str | None] = lambda c, tr: None  # plant step to force now
    buttons: Callable[[Ctx], dict] = lambda c: {}                   # operator buttons for this scan
    done: Callable[[Ctx], bool] = lambda c: False


def _cycles(n: int) -> Callable[[Ctx], bool]:
    return lambda c: c.closes >= n and c.robot_waits_for_mould() and c.plc_state() == "S_MOULDING"


def _alarm(c: Ctx) -> bool:
    return c.s.rob.alarm


def _area_inject(c: Ctx, tr: dict) -> str | None:
    if c.closes >= 1 and c.s.imm.mould == "CLOSING" and "fault:AREA_FREE_stuck_low" in tr and c.after("closing", 2):
        return "fault:AREA_FREE_stuck_low"
    return None


def _access_inject(c: Ctx, tr: dict) -> str | None:
    if "op:gate_open" in tr and "opened" not in c.memo and c.after("unlocked", 3):
        c.memo["opened"] = c.scan
        return "op:gate_open"
    if "op:gate_close" in tr and c.after("open", 6):
        return "op:gate_close"
    return None


def _access_buttons(c: Ctx) -> dict:
    if c.closes == 1 and "req" not in c.memo and c.s.rob.pos == "CUT":
        c.memo["req"] = c.scan
        return {"BTNGATEREQ": True}
    if "opened" in c.memo and not c.s.env.gate_open and "reset" not in c.memo and c.after("closed", 3):
        c.memo["reset"] = c.scan
        return {"BTNRESET": True}
    return {}


def _forced_inject(c: Ctx, tr: dict) -> str | None:
    if (c.closes == 1 and "opened" not in c.memo and "op:gate_open" in tr and c.robot_waits_for_mould()
            and c.s.imm.mould == "CLOSED" and c.plc_state() == "S_MOULDING" and c.after("waiting", 3)):
        c.memo["opened"] = c.scan
        return "op:gate_open"
    if "op:gate_close" in tr and c.after("open", 6):
        return "op:gate_close"
    return None


def _forced_buttons(c: Ctx) -> dict:
    if "opened" in c.memo and not c.s.env.gate_open and "reset" not in c.memo and c.after("closed", 4):
        c.memo["reset"] = c.scan
        return {"BTNRESET": True}
    return {}


CASES = [
    Case("normal_cycles", "three normal cycles", done=_cycles(3)),
    Case("robot_late", "robot still downstream when the mould opens again (slow sprue cutter)",
         dur={"cell:cutter_closed": 60, "rob:timeout": 100}, done=lambda c: c.closes >= 3),
    Case("area_free_lost_during_closing", "\"Mould area free\" monitoring signal lost during the closing stroke",
         scenario="AREA_FREE_stuck_low", inject=_area_inject, done=_alarm),
    Case("access_on_request", "access requested, gate released, opened, closed, reset, one more cycle",
         scenario="gate_request", inject=_access_inject, buttons=_access_buttons, done=_cycles(3)),
    Case("gate_opened_and_reset", "gate opened without request while the robot waits, closed, reset, one more cycle",
         scenario="gate_forced", inject=_forced_inject, buttons=_forced_buttons, done=_cycles(3)),
    Case("conveyor_not_ready", "conveyor never indexes: the robot times out at the nest and reports a fault",
         dur={"cell:conveyor_indexed": None}, done=_alarm),
    Case("closing_watchdog", "the second closing stroke never ends: the watchdog of the PLC expires",
         inject=lambda c, tr: None, done=_alarm),
]


def run_case(plc, case: Case, max_scans: int = 2000) -> dict:
    sysm = System(plc, "A", case.scenario, timeouts=True, close_tmo_scans=WATCHDOG_SCANS)
    c = Ctx(sysm)
    dur = {**DUR, **case.dur}
    since: dict[str, int] = {}
    rows: list[tuple[tuple, tuple]] = []
    events: list[tuple[int, str]] = []
    end_at = None
    while c.scan < max_scans:
        for _ in range(400):                                  # plant steps of this scan
            tr = {}
            for lab, s2 in sysm.transitions(c.s):
                tr.setdefault(lab, s2)
            for lab in list(since):
                if lab not in tr:
                    del since[lab]
            for lab in tr:
                since.setdefault(lab, c.scan)
            if c.s.env.gate_open:
                c.after("open", 0)
            elif "opened" in c.memo:
                c.after("closed", 0)
            if c.s.out[sysm.oi["QGATEUNLOCK"]]:
                c.after("unlocked", 0)
            if c.s.imm.mould == "CLOSING":
                c.after("closing", 0)
            else:
                c.memo.pop("closing", None)
            if c.robot_waits_for_mould():
                c.after("waiting", 0)
            else:
                c.memo.pop("waiting", None)
            fire = case.inject(c, tr)
            if fire is None:
                for lab in tr:
                    if lab.startswith(("plc:", "op:", "fault:")) or lab in ROBOT_NOW:
                        continue
                    d = dur[lab]
                    if case.key == "closing_watchdog" and lab == "imm:close_done" and c.closes >= 1:
                        d = None
                    if d is not None and c.scan - since[lab] >= d:
                        fire = lab
                        break
            if fire is None:
                fire = next((lab for lab in ROBOT_NOW if lab in tr), None)
            if fire is None:
                break
            c.s = tr[fire]
            since.pop(fire, None)
            events.append((c.scan, fire))
            if fire == "imm:close_done":
                c.closes += 1
        else:
            raise RuntimeError(f"{case.key}: plant does not settle in scan {c.scan}")
        d = sysm.sig(c.s)
        extra = {**case.buttons(c), "CFGCLOSETMOSCANS": WATCHDOG_SCANS}
        inputs = sysm._plc_inputs(c.s, d, extra)
        plc2, out2 = plc.scan(c.s.plc, inputs)
        c.s = c.s._replace(plc=plc2, out=out2)
        rows.append((inputs, out2))
        c.scan += 1
        if end_at is None and case.done(c):
            end_at = c.scan + SETTLE
        if end_at is not None and c.scan >= end_at:
            break
    else:
        tail = ", ".join(f"{t}:{lab}" for t, lab in events[-12:])
        raise RuntimeError(f"{case.key}: end condition not reached in {max_scans} scans; state {sysm.describe(c.s)}; "
                           f"last events {tail}")
    out = c.s.out
    return {"key": case.key, "title": case.title, "rows": rows, "events": events, "scans": len(rows),
            "end_state": plc.name_of("S_", out[sysm.oi["STATE"]]), "end_fault": plc.name_of("F_", out[sysm.oi["FAULTCODE"]])
            if out[sysm.oi["FAULTCODE"]] else "none", "cycles": c.closes, "robot_alarm": c.s.rob.alarm}


def run_all(plc) -> list[dict]:
    return [run_case(plc, case) for case in CASES]

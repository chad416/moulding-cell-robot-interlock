"""The cell as communicating state machines at signal level.

Components
  IMM    injection moulding machine: a small state machine written here (mould, shot, ejector). Its interface is
         the EUROMAP 67 signal set (version 1.11); contact numbers are in SIGNALS.
  ROBOT  the TP program file itself, executed by cell/tp.py, plus the tool position and the gripper.
  PLC    the compiled ST block CELL_PLC (cell/plc.py), one scan per step.
  CELL   sprue cutter, conveyor nest, gate with guard locking, operator buttons.

One global step is one of: a machine event, one robot instruction or move event, one PLC scan, one cell-device
event, one operator action, the activation of the fault of the scenario. The order is free (asynchronous
interleaving), so no result depends on a scan time or a move time. The timed run (cell/loop.py) uses the same
`transitions` function and only chooses when each step happens.

Hazard predicates (on the physical state, not on the possibly faulty signals)
  P1  the mould makes a closing movement while the gripper is in the mould area
  P2  the gripper is in the mould area while the mould is not at its open position
"""

from __future__ import annotations

from pathlib import Path
from typing import NamedTuple

import yaml

from .tp import Program, Rob

ROOT = Path(__file__).resolve().parents[1]

# name: (EUROMAP 67 contact or robot I/O, from, to, meaning)
SIGNALS = {
    "ZA6": ("ZA6", "IMM", "PLC", "Mould closed"),
    "ZA7": ("ZA7", "IMM", "PLC", "Mould open position"),
    "ZB2": ("ZB2", "IMM", "PLC", "Enable operation with handling device / robot"),
    "ZB3": ("ZB3", "IMM", "PLC", "Ejector back position"),
    "ZB4": ("ZB4", "IMM", "PLC", "Ejector forward position"),
    "A6": ("A6", "PLC", "IMM", "Enable mould closure"),
    "B3": ("B3", "PLC", "IMM", "Enable ejector back"),
    "B4": ("B4", "PLC", "IMM", "Enable ejector forward"),
    "MAF": ("A3/C3", "ROBOT", "IMM", "Mould area free (one switch contact)"),
    "REQ_CLOSE": ("DO[1]", "ROBOT", "PLC", "request Enable mould closure"),
    "REQ_EJ_FWD": ("DO[2]", "ROBOT", "PLC", "request Enable ejector forward"),
    "REQ_EJ_BACK": ("DO[3]", "ROBOT", "PLC", "request Enable ejector back"),
    "AREA_FREE": ("DO[10]", "ROBOT", "PLC", "monitoring copy of Mould area free"),
    "MOULD_OPEN": ("DI[2]", "PLC", "ROBOT", "mould open, enter"),
    "MOULD_CLOSED": ("DI[3]", "PLC", "ROBOT", "closure acknowledged"),
    "EJ_FWD": ("DI[4]", "PLC", "ROBOT", "ejector forward"),
    "EJ_BACK": ("DI[5]", "PLC", "ROBOT", "ejector back"),
    "CYCLE_ENABLE": ("DI[8]", "PLC", "ROBOT", "next cycle allowed"),
}


class Imm(NamedTuple):
    mould: str       # CLOSED, OPENING, OPEN, CLOSING
    shot: str        # PROCESS (injection, holding, cooling), DONE
    ej: str          # BACK, FWD
    ejd: bool        # ejector sequence done in this cycle


class Env(NamedTuple):
    has_part: bool
    cut_closed: bool
    nest_full: bool
    gate_open: bool


class State(NamedTuple):
    imm: Imm
    rob: Rob
    plc: bytes
    out: tuple       # latched PLC outputs
    env: Env
    fault: bool      # the fault of the scenario is active


class Variant(NamedTuple):
    key: str
    title: str
    program: str = "robot/UNLOAD_IMM.ls"
    use_maf: bool = True         # the machine evaluates the Mould area free contact
    a6_direct: bool = False      # A6 wired straight from the robot DO[1], no PLC gating


VARIANTS = {
    "A": Variant("A", "design: Mould area free contact used, A6 gated by the PLC, four-phase program"),
    "B": Variant("B", "as A, but the Mould area free contact is bridged at the machine", use_maf=False),
    "C": Variant("C", "weakened: contact bridged, A6 straight from the robot output, early-enable program",
                 program="robot/variants/UNLOAD_EARLY.ls", use_maf=False, a6_direct=True),
    "D": Variant("D", "early-enable program inside the full design (contact used, A6 gated by the PLC)",
                 program="robot/variants/UNLOAD_EARLY.ls"),
}


def scenarios() -> dict[str, dict]:
    """Fault list. Each scenario adds at most one fault, which becomes active at any moment and stays."""
    sc: dict[str, dict] = {"none": {"kind": "none", "text": "no fault"}}
    for s in SIGNALS:
        for v in (0, 1):
            sc[f"{s}_stuck_{'high' if v else 'low'}"] = {
                "kind": "stuck", "signal": s, "value": v,
                "text": f"{SIGNALS[s][3]} ({SIGNALS[s][0]}) stuck {'HIGH' if v else 'LOW'}; also covers a lost edge"}
    sc["zone_function_stuck_free"] = {"kind": "zone", "text": "the robot zone function reports free while the gripper "
                                      "is inside: contact A3/C3 and DO[10] both stuck (common cause)"}
    sc["robot_hold"] = {"kind": "hold", "text": "robot stopped mid-path or mid-program, any time, any duration"}
    sc["gate_forced"] = {"kind": "gate_forced", "text": "operator opens the gate without request (no guard locking)"}
    sc["gate_request"] = {"kind": "gate_request", "text": "operator requests access, opens, closes, resets"}
    sc["robot_power_loss"] = {"kind": "robot_off", "text": "robot controller loses power: outputs LOW, contacts open"}
    sc["plc_power_loss"] = {"kind": "plc_off", "text": "cell PLC loses power: all outputs LOW"}
    sc["plc_power_loss_A6_high"] = {"kind": "plc_off_a6", "text": "cell PLC loses power and A6 goes HIGH, as "
                                    "EUROMAP 67 asks for a handling device that is switched off"}
    sc["imm_power_loss"] = {"kind": "imm_off", "text": "machine loses power: no movement, outputs LOW"}
    return sc


class System:
    def __init__(self, plc, variant: str = "A", scenario: str = "none", timeouts: bool = True,
                 close_tmo_scans: int | None = None, layout: str = "side_6axis",
                 plc_faster_than_process: bool = False):
        """timeouts=False switches off the PLC watchdog and the robot WAIT timeouts.
        close_tmo_scans=None (exploration): the watchdog of the closing stroke may expire at any scan from the
        second one on, or never. Each scan runs with cfgCloseTmoScans = 0 (no count) or 1 (count, expire above
        one), chosen freely, so the model contains every real preset. A number (timed run) is the real preset.
        plc_faster_than_process is the one timing assumption that the liveness check needs: the shot does not
        end before the PLC has settled on the current signals (tens of seconds against a scan of
        milliseconds). No safety result uses it."""
        self.plc_fast = plc_faster_than_process
        self.plc = plc
        self.v = VARIANTS[variant]
        self.sc_name = scenario
        self.sc = scenarios()[scenario]
        self.timeouts = timeouts
        self.wd = [0] if not timeouts else ([0, 1] if close_tmo_scans is None else [close_tmo_scans])
        self.prog = Program(ROOT / self.v.program)
        cfg = yaml.safe_load((ROOT / "config" / "cell.yaml").read_text(encoding="utf-8"))
        self.zone = {k: p["zone"] for k, p in cfg["layouts"][layout]["positions"].items()}
        self.oi = {n: i for i, n in enumerate(plc.output_names)}
        self.kind = self.sc["kind"]

    # ---- physical facts -------------------------------------------------------------------------------------
    def inside(self, rob: Rob) -> bool:
        """Gripper in the mould area: at a mould position, or on a move that starts or ends at one."""
        return self.zone[rob.pos] == "MOULD" or (rob.target is not None and self.zone[rob.target] == "MOULD")

    def initial(self) -> State:
        out = tuple(0 for _ in self.plc.output_names)
        return State(Imm("CLOSED", "PROCESS", "BACK", False), self.prog.initial("HOME"), self.plc.initial, out,
                     Env(False, False, False, False), False)

    # ---- signals, with the fault of the scenario applied ----------------------------------------------------
    def sig(self, s: State) -> dict:
        o = lambda n: bool(s.out[self.oi[n.upper()]])
        k, f = self.kind, s.fault
        rob_off, plc_off, imm_off = f and k == "robot_off", f and k in ("plc_off", "plc_off_a6"), f and k == "imm_off"
        free = not self.inside(s.rob)
        d = {
            "ZA6": s.imm.mould == "CLOSED", "ZA7": s.imm.mould == "OPEN", "ZB2": True,
            "ZB3": s.imm.ej == "BACK", "ZB4": s.imm.ej == "FWD",
            "A6": o("qA6_EnableMouldClosure"), "B3": o("qB3_EnableEjectorBack"), "B4": o("qB4_EnableEjectorForward"),
            "MAF": free,
            "REQ_CLOSE": 1 in s.rob.do, "REQ_EJ_FWD": 2 in s.rob.do, "REQ_EJ_BACK": 3 in s.rob.do,
            "AT_CUT": 4 in s.rob.do, "PLACED": 5 in s.rob.do, "FAULT": 6 in s.rob.do, "AT_WAIT": 7 in s.rob.do,
            "AREA_FREE": free,
            "IMM_AUTO": o("qRbImmAuto"), "MOULD_OPEN": o("qRbMouldOpen"), "MOULD_CLOSED": o("qRbMouldClosed"),
            "EJ_FWD": o("qRbEjFwd"), "EJ_BACK": o("qRbEjBack"), "CUT_DONE": o("qRbCutDone"),
            "PLACE_FREE": o("qRbPlaceFree"), "CYCLE_ENABLE": o("qRbCycleEnable"), "PLACE_ACK": o("qRbPlaceAck"),
            "CUT_CLOSE": o("qCutClose"), "CONV_INDEX": o("qConvIndex"), "GATE_UNLOCK": o("qGateUnlock"),
        }
        if imm_off:
            for n in ("ZA6", "ZA7", "ZB2", "ZB3", "ZB4"):
                d[n] = False
        if rob_off:
            for n in ("MAF", "REQ_CLOSE", "REQ_EJ_FWD", "REQ_EJ_BACK", "AT_CUT", "PLACED", "FAULT",
                      "AT_WAIT", "AREA_FREE"):
                d[n] = False
        if plc_off:
            for n in ("A6", "B3", "B4", "IMM_AUTO", "MOULD_OPEN", "MOULD_CLOSED", "EJ_FWD", "EJ_BACK", "CUT_DONE",
                      "PLACE_FREE", "CYCLE_ENABLE", "PLACE_ACK", "CUT_CLOSE", "CONV_INDEX", "GATE_UNLOCK"):
                d[n] = False
            if k == "plc_off_a6":
                d["A6"] = True
        if self.v.a6_direct:
            d["A6"] = d["REQ_CLOSE"]
        if f and k == "stuck":
            d[self.sc["signal"]] = bool(self.sc["value"])
        if f and k == "zone":
            d["MAF"] = d["AREA_FREE"] = True
        d["SAFE"] = not s.env.gate_open            # safety devices and gate, contact signal to machine and robot
        d["MAF_OK"] = d["MAF"] or not self.v.use_maf   # what the machine evaluates
        return d

    def closing_permitted(self, s: State, d: dict) -> bool:
        return d["A6"] and d["MAF_OK"] and d["SAFE"] and not (s.fault and self.kind == "imm_off")

    def hazards(self, s: State) -> tuple[bool, bool]:
        ins = self.inside(s.rob)
        d = self.sig(s)
        p1 = ins and s.imm.mould == "CLOSING" and self.closing_permitted(s, d)
        p2 = ins and s.imm.mould != "OPEN"
        return p1, p2

    def _plc_inputs(self, s: State, d: dict, extra: dict | None = None) -> tuple:
        e = s.env
        vals = {
            "IZA6_MOULDCLOSED": d["ZA6"], "IZA7_MOULDOPENPOSITION": d["ZA7"], "IZB2_ENABLEOPERATION": d["ZB2"],
            "IZB3_EJECTORBACK": d["ZB3"], "IZB4_EJECTORFORWARD": d["ZB4"], "ISAFETYOK": d["SAFE"],
            "RBREQCLOSE": d["REQ_CLOSE"], "RBREQEJFWD": d["REQ_EJ_FWD"], "RBREQEJBACK": d["REQ_EJ_BACK"],
            "RBATCUT": d["AT_CUT"], "RBPLACED": d["PLACED"], "RBFAULT": d["FAULT"], "RBATWAIT": d["AT_WAIT"],
            "RBAREAFREE": d["AREA_FREE"], "ICUTCLOSED": e.cut_closed, "ICUTOPEN": not e.cut_closed,
            "INESTFREE": not e.nest_full, "IGATECLOSEDLOCKED": not e.gate_open and not d["GATE_UNLOCK"],
            "BTNGATEREQ": False, "BTNRESET": False, "CFGCLOSETMOSCANS": self.wd[0],
        }
        if extra:
            vals.update(extra)
        return tuple(int(vals[n]) for n in self.plc.input_names)

    def _plc_pending(self, s: State, d: dict) -> bool:
        return self.plc.scan(s.plc, self._plc_inputs(s, d)) != (s.plc, s.out)

    # ---- transitions ----------------------------------------------------------------------------------------
    def transitions(self, s: State) -> list[tuple[str, State]]:
        d = self.sig(s)
        k, f = self.kind, s.fault
        t: list[tuple[str, State]] = []
        m = s.imm

        # machine
        if not (f and k == "imm_off"):
            move_ok = d["SAFE"] and d["MAF_OK"]
            if m.mould == "CLOSED" and m.shot == "PROCESS" and not (self.plc_fast and self._plc_pending(s, d)):
                t.append(("imm:process_done", s._replace(imm=m._replace(shot="DONE"))))
            if m.mould == "CLOSED" and m.shot == "DONE" and move_ok:
                t.append(("imm:open_start", s._replace(imm=m._replace(mould="OPENING"))))
            if m.mould == "OPENING" and move_ok:
                t.append(("imm:open_done", s._replace(imm=m._replace(mould="OPEN"))))
            if m.mould == "OPEN" and m.ej == "BACK" and d["B4"] and d["SAFE"]:
                t.append(("imm:ej_fwd_done", s._replace(imm=m._replace(ej="FWD"))))
            if m.mould == "OPEN" and m.ej == "FWD" and d["B3"] and d["SAFE"]:
                t.append(("imm:ej_back_done", s._replace(imm=m._replace(ej="BACK", ejd=True))))
            if m.mould == "OPEN" and m.ejd and m.ej == "BACK" and self.closing_permitted(s, d):
                t.append(("imm:close_start", s._replace(imm=m._replace(mould="CLOSING"))))
            if m.mould == "CLOSING" and self.closing_permitted(s, d):
                t.append(("imm:close_done", s._replace(imm=Imm("CLOSED", "PROCESS", m.ej, False))))

        # robot: stopped by the safety circuit (gate), by a hold, or without power
        stopped = (not d["SAFE"]) or (f and k in ("hold", "robot_off"))
        if not stopped:
            def read(kind, n):
                if kind == "DI":
                    return d[DI[n]]
                if kind == "RI":
                    return s.env.has_part
                return d["AREA_FREE"] if n == 10 else (n in s.rob.do)
            for kind, r2 in self.prog.steps(s.rob, read, self.timeouts):
                t.append((f"rob:{kind}", s._replace(rob=r2)))

        # gripper
        e = s.env
        if 1 in s.rob.ro and not e.has_part and s.rob.pos == "GRIP" and s.rob.target is None:
            t.append(("cell:part_gripped", s._replace(env=e._replace(has_part=True))))
        if 1 not in s.rob.ro and e.has_part:
            placed = s.rob.pos == "PLACE" and s.rob.target is None
            t.append(("cell:part_released", s._replace(env=e._replace(has_part=False, nest_full=e.nest_full or placed))))

        # PLC scan
        if not (f and k in ("plc_off", "plc_off_a6")):
            buttons = [("", {})]
            if k == "gate_request":
                buttons += [("+gate_request", {"BTNGATEREQ": True}), ("+reset", {"BTNRESET": True})]
            if k == "gate_forced":
                buttons += [("+reset", {"BTNRESET": True})]
            for tag, extra in buttons:
                for wd in self.wd:
                    plc2, out2 = self.plc.scan(s.plc, self._plc_inputs(s, d, {**extra, "CFGCLOSETMOSCANS": wd}))
                    t.append((f"plc:scan{tag}", s._replace(plc=plc2, out=out2)))

        # cell devices
        if d["CUT_CLOSE"] and not e.cut_closed and d["SAFE"]:
            t.append(("cell:cutter_closed", s._replace(env=e._replace(cut_closed=True))))
        if not d["CUT_CLOSE"] and e.cut_closed:
            t.append(("cell:cutter_open", s._replace(env=e._replace(cut_closed=False))))
        if d["CONV_INDEX"] and e.nest_full and d["SAFE"]:
            t.append(("cell:conveyor_indexed", s._replace(env=e._replace(nest_full=False))))

        # operator and gate
        if k == "gate_request":
            if d["GATE_UNLOCK"] and not e.gate_open:
                t.append(("op:gate_open", s._replace(env=e._replace(gate_open=True))))
        if k == "gate_forced" and not e.gate_open:
            t.append(("op:gate_open", s._replace(env=e._replace(gate_open=True))))
        if e.gate_open:
            t.append(("op:gate_close", s._replace(env=e._replace(gate_open=False))))

        # fault of the scenario
        if k in ("stuck", "zone", "hold", "robot_off", "plc_off", "plc_off_a6", "imm_off"):
            if not f:
                t.append((f"fault:{self.sc_name}", s._replace(fault=True)))
            elif k == "hold":
                t.append(("fault:release", s._replace(fault=False)))
        return [(lab, s2) for lab, s2 in t if s2 != s]

    # ---- display --------------------------------------------------------------------------------------------
    def describe(self, s: State) -> str:
        st = s.out[self.oi["STATE"]]
        pos = s.rob.pos if s.rob.target is None else f"{s.rob.pos}>{s.rob.target}"
        return (f"mould {s.imm.mould:<8} ej {s.imm.ej:<4} | robot {pos:<18} line {self.prog.where(s.rob):<48}"
                f" | PLC {self.plc.name_of('S_', st)}")


DI = {1: "IMM_AUTO", 2: "MOULD_OPEN", 3: "MOULD_CLOSED", 4: "EJ_FWD", 5: "EJ_BACK", 6: "CUT_DONE", 7: "PLACE_FREE",
      8: "CYCLE_ENABLE", 9: "PLACE_ACK"}

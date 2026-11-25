"""Timed in-the-loop run: compiled PLC block + TP program + machine model, with real durations.

The run uses the same transition function as the state-space exploration (cell/system.py). This module only
decides when each enabled step happens:
  machine and cell-device events   after they have been enabled for their duration (cycle and device times)
  robot moves                      after the move time from cell/motion.py; WAIT x(sec) and WAIT timeouts by the clock
  other robot instructions         at the robot I/O scan
  PLC scan                         every scan period
"""

from __future__ import annotations

from .cooling import cycle_breakdown
from .motion import six_axis_move_times
from .system import System

TRACE_SIGNALS = ["ZA6", "ZA7", "MOULD_OPEN", "AREA_FREE", "MAF", "REQ_EJ_FWD", "B4", "ZB4", "REQ_EJ_BACK", "B3",
                 "ZB3", "REQ_CLOSE", "A6", "MOULD_CLOSED"]


def run_timed(plc, cfg: dict, layout: str = "side_6axis", cycles: int = 3, variant: str = "A",
              tick_ms: int = 2, t_max_s: float = 400.0) -> dict:
    scan_ms = round(cfg["plc"]["scan_s"] * 1000)
    rob_ms = round(cfg["robot_6axis"]["io_scan_s"] * 1000)
    sysm = System(plc, variant, "none", timeouts=True, layout=layout,
                  close_tmo_scans=round(cfg["plc"]["close_timeout_s"] / cfg["plc"]["scan_s"]))
    moves = six_axis_move_times(cfg, layout)
    c, g, dev = cfg["cycle"], cfg["gripper"], cfg["cell_devices"]
    bd = cycle_breakdown(cfg, 0.0)
    fixed = {
        "imm:process_done": c["injection_s"] + bd["cooling_s"] + c["dry_overhead_s"],
        "imm:open_start": 0.0, "imm:open_done": c["mould_open_s"], "imm:ej_fwd_done": c["ejector_fwd_s"],
        "imm:ej_back_done": c["ejector_back_s"], "imm:close_start": 0.0, "imm:close_done": c["mould_close_s"],
        "cell:part_gripped": g["vacuum_on_s"], "cell:part_released": g["release_s"],
        "cell:cutter_closed": dev["cutter_close_s"], "cell:cutter_open": dev["cutter_open_s"],
        "cell:conveyor_indexed": dev["conveyor_index_s"], "rob:timeout": sysm.prog.wait_timeout_s,
    }

    def duration(label: str, s) -> float:
        if label == "rob:move_done":
            return moves[(s.rob.pos, s.rob.target)]
        if label == "rob:delay":
            return sysm.prog.ins[s.rob.pc].a
        return fixed[label]

    s = sysm.initial()
    since: dict[str, int] = {}
    states, events, trace = [s], [], []
    last_sig = None
    t = 0
    opens = []                       # times of ZA7 rising
    while t <= t_max_s * 1000 and len(opens) <= cycles:
        progressed = True
        while progressed:
            progressed = False
            tr = dict(sysm.transitions(s))
            for lab in list(since):
                if lab not in tr:
                    del since[lab]
            for lab in tr:
                since.setdefault(lab, t)
            fire = None
            for lab in tr:
                if lab.startswith("plc:") or lab in ("rob:instr", "rob:wait_ok", "rob:move_start", "rob:alarm"):
                    continue
                if t - since[lab] >= duration(lab, s) * 1000 - 1e-9:
                    fire = lab
                    break
            if fire is None and t % rob_ms == 0:
                fire = next((l for l in ("rob:instr", "rob:wait_ok", "rob:move_start", "rob:alarm") if l in tr), None)
            if fire is not None:
                s = tr[fire]
                since.pop(fire, None)
                states.append(s)
                events.append((t / 1000.0, fire))
                progressed = True
        if t % scan_ms == 0:
            tr = dict(sysm.transitions(s))
            if "plc:scan" in tr:
                s = tr["plc:scan"]
                states.append(s)
        d = sysm.sig(s)
        sig = tuple(int(d[n]) for n in TRACE_SIGNALS)
        if sig != last_sig:
            if last_sig is not None and sig[1] and not last_sig[1]:
                opens.append(t / 1000.0)
            trace.append((t / 1000.0, sig, s.imm.mould, s.rob.pos if s.rob.target is None else f"{s.rob.pos}>{s.rob.target}"))
            last_sig = sig
        t += tick_ms

    def edges(name: str, rising: bool = True) -> list[float]:
        i, out, prev = TRACE_SIGNALS.index(name), [], None
        for tt, sg, *_ in trace:
            if prev is not None and bool(sg[i]) != bool(prev) and bool(sg[i]) == rising:
                out.append(tt)
            prev = sg[i]
        return out

    za7_up, a6_up, za7_dn = edges("ZA7"), edges("A6"), edges("ZA7", False)
    per_cycle = []
    for k in range(1, len(za7_up) - 1):          # skip the first cycle (robot starts from HOME)
        t0 = za7_up[k]
        ta6 = next(x for x in a6_up if x > t0)
        tdn = next(x for x in za7_dn if x > t0)
        per_cycle.append({"t_open": t0, "removal_s": ta6 - t0, "mould_open_s": tdn - t0,
                          "cycle_s": za7_up[k + 1] - t0})
    hazards = [sysm.hazards(x) for x in states]
    return {"layout": layout, "variant": variant, "signals": TRACE_SIGNALS, "trace": trace, "events": events,
            "states": states, "system": sysm, "cycles": per_cycle,
            "p1_seen": any(h[0] for h in hazards), "p2_seen": any(h[1] for h in hazards),
            "alarm": s.rob.alarm, "plc_state": s.out[sysm.oi["STATE"]], "t_end": t / 1000.0}


def observable(s) -> tuple:
    """State without the PLC's internal memory (its outputs and state number stay)."""
    return (s.imm, s.rob, s.out, s.env)

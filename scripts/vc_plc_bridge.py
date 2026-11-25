"""Machine model and compiled PLC block for a run against a robot controller (Linux side of scripts/vc_run.py).

Reads one JSON object per line on stdin, answers with one JSON object per line on stdout.
  in : {"t": seconds, "do": [numbers that are ON], "ro": [...], "pos": position name, "block": [event labels]}
  out: {"di": {name: 0|1}, "ri1": 0|1, "sig": {...}, "mould": ..., "plc": state name, "events": [[t, label], ...]}

The robot is outside this process: its outputs and its position come in with every message. Everything else is
the repository's own code: the machine and cell-device model of cell/system.py and the PLC block compiled with
matiec, scanned every 10 ms of the time that has passed. "block" holds back machine events for a test (for
example the ejector that never moves forward).

    python3 -u -m scripts.vc_plc_bridge [process_time_s]
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import yaml

from cell.plc import Plc
from cell.system import DI, System
from cell.tp import Rob

ROOT = Path(__file__).resolve().parents[1]
TRACE = ["ZA7", "MOULD_OPEN", "MAF", "REQ_EJ_FWD", "B4", "ZB4", "REQ_EJ_BACK", "B3", "ZB3", "REQ_CLOSE", "A6", "ZA6",
         "MOULD_CLOSED", "AT_CUT", "CUT_DONE", "PLACED", "PLACE_ACK", "PLACE_FREE", "CYCLE_ENABLE", "IMM_AUTO", "FAULT"]


def main() -> None:
    cfg = yaml.safe_load((ROOT / "config" / "cell.yaml").read_text(encoding="utf-8"))
    process_s = float(sys.argv[1]) if len(sys.argv) > 1 else 8.0
    c, g, dev = cfg["cycle"], cfg["gripper"], cfg["cell_devices"]
    scan = cfg["plc"]["scan_s"]
    dur = {"imm:process_done": process_s, "imm:open_start": 0.0, "imm:open_done": c["mould_open_s"],
           "imm:ej_fwd_done": c["ejector_fwd_s"], "imm:ej_back_done": c["ejector_back_s"], "imm:close_start": 0.0,
           "imm:close_done": c["mould_close_s"], "cell:part_gripped": g["vacuum_on_s"],
           "cell:part_released": g["release_s"], "cell:cutter_closed": dev["cutter_close_s"],
           "cell:cutter_open": dev["cutter_open_s"], "cell:conveyor_indexed": dev["conveyor_index_s"]}
    plc = Plc()
    sysm = System(plc, "A", "none", close_tmo_scans=round(cfg["plc"]["close_timeout_s"] / scan))
    s = sysm.initial()
    since: dict[str, float] = {}
    now = None
    for line in sys.stdin:
        line = line.strip().lstrip("﻿")
        if not line:
            continue
        msg = json.loads(line)
        if msg.get("quit"):
            break
        t = float(msg["t"])
        rob = Rob(0, msg["pos"], None, frozenset(msg["do"]), frozenset(msg["ro"]), False)
        s = s._replace(rob=rob)
        block = set(msg.get("block", []))
        steps = 1 if now is None else max(1, min(300, round((t - now) / scan)))
        start = t - steps * scan
        events = []
        for k in range(1, steps + 1):
            tk = start + k * scan
            for _ in range(50):
                tr = {}
                for lab, s2 in sysm.transitions(s):
                    if lab.startswith(("imm:", "cell:")) and lab not in block:
                        tr.setdefault(lab, s2)
                for lab in list(since):
                    if lab not in tr:
                        del since[lab]
                for lab in tr:
                    since.setdefault(lab, tk)
                fire = None
                for lab in tr:
                    if tk - since[lab] >= dur[lab] - 1e-9:
                        fire = lab
                        break
                if fire is None:
                    break
                s = tr[fire]._replace(rob=rob)
                since.pop(fire, None)
                events.append([round(tk, 3), fire])
            d = sysm.sig(s)
            plc2, out2 = plc.scan(s.plc, sysm._plc_inputs(s, d))
            s = s._replace(plc=plc2, out=out2)
        now = t
        d = sysm.sig(s)
        p1, p2 = sysm.hazards(s)
        out = {"di": {name: int(d[name]) for name in DI.values()}, "di_num": {n: int(d[name]) for n, name in DI.items()},
               "ri1": int(s.env.has_part), "sig": {n: int(d[n]) for n in TRACE}, "mould": s.imm.mould, "ej": s.imm.ej,
               "plc": plc.name_of("S_", s.out[sysm.oi["STATE"]]),
               "plc_fault": plc.name_of("F_", s.out[sysm.oi["FAULTCODE"]]) if s.out[sysm.oi["FAULTCODE"]] else "none",
               "p1": int(p1), "p2": int(p2), "events": events}
        sys.stdout.write(json.dumps(out) + "\n")
        sys.stdout.flush()


if __name__ == "__main__":
    main()

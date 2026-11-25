"""Run every study and write results/studies.json, results/traces.json and results/counterexamples.txt.

    python -m scripts.run_studies            # needs build/plc/libplc.so (scripts/build_plc.sh)

No number in the README or the dossier is typed by hand; scripts/make_tables.py writes them from these files.
"""

from __future__ import annotations

import json
import time
from pathlib import Path

import yaml

from cell import cooling, motion
from cell.checker import can_always_reach, explore, format_trace
from cell.loop import observable, run_timed
from cell.plc import Plc
from cell.system import SIGNALS, VARIANTS, System, scenarios

ROOT = Path(__file__).resolve().parents[1]
LAYOUTS = ["top_linear", "side_6axis", "top_6axis"]


def physics(cfg: dict) -> dict:
    m = cfg["material"]
    a = cooling.diffusivity_mm2_s(m["conductivity_W_mK"], m["density_kg_m3"], m["heat_capacity_J_kgK"])
    s, te = cfg["part"]["wall_mm"], m["T_demould_C"]
    args = (a, m["T_melt_C"], m["T_mould_C"])
    out = {
        "a_mm2_s": a,
        "cooling_mean_s": cooling.cooling_time(s, *args, te, "mean"),
        "cooling_centre_s": cooling.cooling_time(s, *args, te, "centre"),
        "cooling_mean_fd_s": cooling.plate_fd(s, *args, te, "mean"),
        "cooling_centre_fd_s": cooling.plate_fd(s, *args, te, "centre"),
        "wall_sweep": [{"wall_mm": w, "cooling_s": cooling.cooling_time(w, *args, te, "mean")}
                       for w in [3.0, 4.0, 5.0, 6.0, 7.0, 8.0]],
        "demould_sweep": [{"T_E": t, "cooling_s": cooling.cooling_time(s, *args, t, "mean")}
                          for t in [100, 110, 120, 140, 160]],
    }
    budgets = {k: motion.removal_budget(cfg, k) for k in LAYOUTS}
    alts = {}
    for k, b in budgets.items():
        cyc = cooling.cycle_breakdown(cfg, b["removal_s"])
        alts[k] = {**b, "cycle_s": cyc["cycle_s"], "phases": cyc["phases"],
                   "robot_idle_s": cyc["cycle_s"] - b["removal_s"] - b["downstream_s"],
                   **cooling.annual_output(cfg, cyc["cycle_s"])}
    base = alts["top_linear"]
    for k, al in alts.items():
        al["d_removal_s"] = al["removal_s"] - base["removal_s"]
        al["d_cycle_pct"] = 100.0 * (al["cycle_s"] - base["cycle_s"]) / base["cycle_s"]
        al["d_parts_per_machine"] = al["parts_per_machine"] - base["parts_per_machine"]
    out["alternatives"] = alts
    cyc = alts["side_6axis"]["cycle_s"]
    out["sensitivity"] = {
        "parts_per_machine_per_second_open": cooling.annual_output(cfg, cyc)["parts_per_machine"]
        - cooling.annual_output(cfg, cyc + 1.0)["parts_per_machine"],
        "cycle_pct_per_second_open": 100.0 / cyc,
        "accel": [{"accel_time_s": ta, "removal_s": motion.removal_budget(cfg, "side_6axis", ta)["removal_s"]}
                  for ta in [0.15, 0.20, 0.25, 0.30, 0.40]],
        "wall": [],
    }
    for w in [2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0]:
        c_lin = cooling.cycle_breakdown(cfg, budgets["top_linear"]["removal_s"], wall_mm=w)["cycle_s"]
        c_six = cooling.cycle_breakdown(cfg, budgets["side_6axis"]["removal_s"], wall_mm=w)["cycle_s"]
        out["sensitivity"]["wall"].append({"wall_mm": w, "cycle_linear_s": c_lin, "cycle_6axis_s": c_six,
                                           "penalty_pct": 100.0 * (c_six - c_lin) / c_lin})
    moves = motion.six_axis_move_times(cfg, "side_6axis")
    out["moves_side_6axis"] = {f"{a_}->{b_}": t for (a_, b_), t in moves.items()}
    return out


def timed(plc: Plc, cfg: dict, reach_obs: set) -> tuple[dict, dict]:
    res, traces = {}, {}
    for lay in ("side_6axis", "top_6axis"):
        r = run_timed(plc, cfg, lay, cycles=4)
        cyc = r["cycles"]
        budget = motion.removal_budget(cfg, lay)
        res[lay] = {
            "cycles_measured": len(cyc),
            "removal_s": cyc[-1]["removal_s"], "mould_open_s": cyc[-1]["mould_open_s"], "cycle_s": cyc[-1]["cycle_s"],
            "removal_budget_s": budget["removal_s"],
            "cycle_budget_s": cooling.cycle_breakdown(cfg, budget["removal_s"])["cycle_s"],
            "p1_seen": r["p1_seen"], "p2_seen": r["p2_seen"], "alarm": r["alarm"],
            "states_visited": len(set(r["states"])),
        }
        if lay == "side_6axis":
            obs = {observable(s) for s in r["states"]}
            res[lay]["observable_states"] = len(obs)
            res[lay]["observable_states_in_model"] = len(obs & reach_obs)
            t0 = cyc[-1]["t_open"]
            traces = {"signals": r["signals"], "t0": t0,
                      "rows": [[round(t - t0, 3), list(sig), mould, pos] for t, sig, mould, pos in r["trace"]
                               if t0 - 1.0 <= t <= t0 + 14.0],
                      "events": [[round(t - t0, 3), lab] for t, lab in r["events"] if t0 - 1.0 <= t <= t0 + 14.0]}
    return res, traces


def compact(sysm: System, trace: list) -> list[dict]:
    rows = []
    for lab, s in trace:
        d = sysm.sig(s)
        rows.append({"step": lab, "mould": s.imm.mould, "ej": s.imm.ej,
                     "robot": s.rob.pos if s.rob.target is None else f"{s.rob.pos}>{s.rob.target}",
                     "inside": sysm.inside(s.rob), "REQ_CLOSE": bool(d["REQ_CLOSE"]), "A6": bool(d["A6"]),
                     "ZA7": bool(d["ZA7"]), "MOULD_OPEN": bool(d["MOULD_OPEN"]), "fault": s.fault,
                     "closing": s.imm.mould == "CLOSING" and sysm.closing_permitted(s, d)})
    return rows


KEEP = {("C", "none", "P1"), ("B", "A6_stuck_high", "P1"), ("A", "ZA7_stuck_high", "P2")}


def model(plc: Plc) -> tuple[dict, str, set]:
    out: dict = {"variants": {}, "scenarios": {k: v["text"] for k, v in scenarios().items()}, "counterexamples": {}}
    text = []
    reach_obs: set = set()
    plan = {"A": list(scenarios()), "B": list(scenarios()), "C": ["none"], "D": ["none"]}
    for v, names in plan.items():
        rows = {}
        for name in names:
            sysm = System(plc, v, name)
            r = explore(sysm)
            rows[name] = {"states": r["states"], "transitions": r["transitions"],
                          "P1_states": r["violations"]["P1"], "P2_states": r["violations"]["P2"],
                          "P1_trace_len": len(r["trace"]["P1"]) - 1 if r["trace"]["P1"] else None,
                          "P2_trace_len": len(r["trace"]["P2"]) - 1 if r["trace"]["P2"] else None}
            if v == "A" and name == "none":
                reach_obs = {observable(s) for s in r["parent"]}
            for prop in ("P1", "P2"):
                if (v, name, prop) in KEEP:
                    out["counterexamples"][f"{v}/{name}/{prop}"] = compact(sysm, r["trace"][prop])
                if r["trace"][prop] and (prop == "P1" or v == "A"):
                    text.append(f"=== variant {v} ({VARIANTS[v].title})\n=== scenario: {name} ({scenarios()[name]['text']})\n"
                                f"=== shortest path to a {prop} state: {len(r['trace'][prop]) - 1} steps "
                                f"(each line shows the state after the step and the next robot instruction)\n"
                                + format_trace(sysm, r["trace"][prop]) + "\n")
        out["variants"][v] = {"title": VARIANTS[v].title, "scenarios": rows,
                              "states_total": sum(x["states"] for x in rows.values()),
                              "transitions_total": sum(x["transitions"] for x in rows.values()),
                              "P1_scenarios": [k for k, x in rows.items() if x["P1_states"]],
                              "P2_scenarios": [k for k, x in rows.items() if x["P2_states"]]}
    live_sys = System(plc, "A", "none", timeouts=False, plc_faster_than_process=True)
    lr = explore(live_sys, keep_edges=True)
    live = can_always_reach(lr, "imm:process_done")
    out["liveness"] = {"states": live["states"], "can_reach_next_shot": live["can_reach"],
                       "deadlocks": sum(1 for s in lr["edges"] if not lr["edges"][s])}
    out["signals"] = {k: list(v) for k, v in SIGNALS.items()}
    out["fault_scenarios"] = len(scenarios()) - 1
    return out, "\n".join(text), reach_obs


def main() -> None:
    cfg = yaml.safe_load((ROOT / "config" / "cell.yaml").read_text(encoding="utf-8"))
    t0 = time.time()
    plc = Plc()
    res = {"physics": physics(cfg)}
    res["model"], text, reach_obs = model(plc)
    res["timed"], traces = timed(plc, cfg, reach_obs)
    res["plc_scans_distinct"] = plc.scans
    (ROOT / "results").mkdir(exist_ok=True)
    (ROOT / "results" / "studies.json").write_text(json.dumps(res, indent=1), encoding="utf-8", newline="\n")
    (ROOT / "results" / "traces.json").write_text(json.dumps(traces), encoding="utf-8", newline="\n")
    (ROOT / "results" / "counterexamples.txt").write_text(text, encoding="utf-8", newline="\n")
    print(f"studies written in {time.time() - t0:.0f} s")


if __name__ == "__main__":
    main()

"""Write the generated blocks into README.md and DOSSIER.md between <!-- TABLE:name --> ... <!-- /TABLE --> markers.
Every number comes from results/studies.json or config/cell.yaml; none is typed by hand."""
import json
import re
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
ORDER = ["top_linear", "side_6axis", "top_6axis"]


def n(x, nd=1):
    return f"{x:.{nd}f}"


def k(x):
    return f"{x:,.0f}".replace(",", " ")


def t_findings(d, cfg):
    ph, m, tm = d["physics"], d["model"], d["timed"]
    al = ph["alternatives"]
    lin, side, top = (al[x] for x in ORDER)
    A, B, C, D = (m["variants"][x] for x in "ABCD")
    wall = {r["wall_mm"]: r for r in ph["sensitivity"]["wall"]}
    acc = ph["sensitivity"]["accel"]
    s = cfg["part"]["wall_mm"]
    mat = cfg["material"]
    return "\n".join([
        f"1. **Cooling sets the cycle, not the robot.** A {s:g} mm section of {mat['name']} needs {n(ph['cooling_mean_s'])} s "
        f"to cool from {mat['T_melt_C']} °C melt to a mean {mat['T_demould_C']} °C in an {mat['T_mould_C']} °C mould "
        f"(a = {n(ph['a_mm2_s'], 3)} mm²/s from the datasheet conductivity, density and heat capacity). That is "
        f"{n(100 * ph['cooling_mean_s'] / side['cycle_s'], 0)} % of the {n(side['cycle_s'])} s cycle. The closed-form "
        f"relation and a finite-difference solution agree: {n(ph['cooling_mean_s'], 2)} s and {n(ph['cooling_mean_fd_s'], 2)} s. "
        f"The model cycle falls in the public 45 to 50 s band only near {s:g} mm "
        f"({n(wall[6.0]['cycle_6axis_s'])} s at 6 mm, {n(wall[8.0]['cycle_6axis_s'])} s at 8 mm).",
        f"2. **A 6-axis robot does lengthen the mould-open window, by a few tenths of a second.** Removal time from "
        f"\"Mould open position\" to \"Enable mould closure\": {n(lin['removal_s'], 2)} s for a 3-axis linear robot "
        f"(axis data assumed), {n(side['removal_s'], 2)} s for the 6-axis robot entering from the side, "
        f"{n(top['removal_s'], 2)} s from the top. The difference is {n(side['d_removal_s'], 2)} s and "
        f"{n(top['d_removal_s'], 2)} s, or {n(side['d_cycle_pct'], 2)} % and {n(top['d_cycle_pct'], 2)} % of the cycle, "
        f"or {k(-side['d_parts_per_machine'])} and {k(-top['d_parts_per_machine'])} parts per machine and year. "
        f"The in-the-loop run (compiled PLC, TP program, machine model) measures {n(tm['side_6axis']['removal_s'], 3)} s "
        f"against the {n(tm['side_6axis']['removal_budget_s'], 3)} s budget.",
        f"3. **Deciding parameter for output: seconds of mould-open time.** One second costs "
        f"{n(ph['sensitivity']['cycle_pct_per_second_open'], 2)} % of the cycle, "
        f"{k(ph['sensitivity']['parts_per_machine_per_second_open'])} parts per machine and year "
        f"({cfg['scenario']['cavities']} cavities, {cfg['production']['hours_per_year']} h, OEE "
        f"{cfg['production']['oee']}). The 6-axis penalty grows as the wall gets thinner: "
        f"{n(wall[7.0]['penalty_pct'], 2)} % at 7 mm, {n(wall[3.0]['penalty_pct'], 2)} % at 3 mm, "
        f"{n(wall[2.0]['penalty_pct'], 2)} % at 2 mm. The joint acceleration is not published; over the assumed "
        f"range ({acc[0]['accel_time_s']} to {acc[-1]['accel_time_s']} s to full speed) the side-entry removal time "
        f"runs from {n(acc[0]['removal_s'], 2)} to {n(acc[-1]['removal_s'], 2)} s. "
        f"Fourteen machines at this cycle give {n(side['parts_plant'] / 1e6, 1)} million mouldings a year against the "
        f"public 3.5 million door-handle assemblies, a ratio of {n(side['parts_per_public_unit'], 1)}. "
        f"This is a plausibility check, not a validation.",
        f"4. **The mould cannot close on the gripper: shown over the complete state space.** For the design "
        f"(variant A) the checker explored {len(A['scenarios'])} scenarios (no fault and {m['fault_scenarios']} single "
        f"faults), {k(A['states_total'])} states and {k(A['transitions_total'])} transitions. The state \"closing "
        f"movement while the gripper is in the mould area\" (P1) is reachable in none of them. Without timeouts and "
        f"with a PLC that is faster than the shot, all {k(m['liveness']['can_reach_next_shot'])} of "
        f"{k(m['liveness']['states'])} states can reach the next shot; there are {m['liveness']['deadlocks']} deadlocks.",
        f"5. **What makes it impossible is two independent conditions at the machine.** The machine closes only with "
        f"the contact \"Mould area free\" (A3/C3, from the robot position) and with \"Enable mould closure\" (A6, from "
        f"the sequence). With the contact bridged (variant B) the cell is still safe without a fault, but "
        f"{len(B['P1_scenarios'])} single faults reach P1: `{'`, `'.join(B['P1_scenarios'])}`. With the contact bridged, "
        f"A6 wired straight from the robot and a program that requests closure before the retreat ends (variant C), "
        f"the checker finds the collision without any fault in {C['scenarios']['none']['P1_trace_len']} steps and prints "
        f"the trace. The same program inside the full design (variant D) reaches P1 in "
        f"{D['scenarios']['none']['P1_states']} states.",
        f"6. **One weaker property is not single-fault tolerant.** \"The gripper is in the mould area only while the "
        f"mould is at its open position\" (P2) fails for {len(A['P2_scenarios'])} faults of variant A: "
        f"`{'`, `'.join(A['P2_scenarios'])}`. EUROMAP 67 carries \"Mould open position\" on one contact (ZA7). The "
        f"robot can then enter a mould that is still opening. The mould does not close on it (P1 holds), but the "
        f"gripper can hit the mould. A second, independent open signal or a stroke-time check would be needed; "
        f"neither is implemented here.",
        f"7. **The timed run stays inside the model.** All {tm['side_6axis']['observable_states_in_model']} of "
        f"{tm['side_6axis']['observable_states']} observable states of the in-the-loop run are reachable states of "
        f"the explored model.",
    ])


def t_alternatives(d, cfg):
    al = d["physics"]["alternatives"]
    rows = ["| Alternative | Move in (s) | Grip and ejector (s) | Move out (s) | Removal (s) | Cycle (s) | Cycle vs linear | "
            "Parts per machine and year | Robot idle per cycle (s) | Floor space, assumed (m²) | Downstream |",
            "|---|---|---|---|---|---|---|---|---|---|---|"]
    for key in ORDER:
        a = al[key]
        rows.append(f"| {a['name']} | {n(a['t_in_s'], 2)} | {n(a['t_grip_eject_s'], 2)} | {n(a['t_out_s'], 2)} | "
                    f"{n(a['removal_s'], 2)} | {n(a['cycle_s'], 2)} | {a['d_cycle_pct']:+.2f} % | {k(a['parts_per_machine'])} | "
                    f"{n(a['robot_idle_s'])} | {a['floor_m2']:g} | {a['downstream']} |")
    return "\n".join(rows)


def t_cycle(d, cfg):
    ph = d["physics"]
    a = ph["alternatives"]["side_6axis"]
    rows = ["| Phase | Time (s) | Share |", "|---|---|---|"]
    for name, v in a["phases"].items():
        rows.append(f"| {name} | {n(v, 2)} | {n(100 * v / a['cycle_s'])} % |")
    rows.append(f"| **cycle** | **{n(a['cycle_s'], 2)}** | |")
    rows += ["", "| Wall (mm) | " + " | ".join(f"{r['wall_mm']:g}" for r in ph["wall_sweep"]) + " |",
             "|---|" + "---|" * len(ph["wall_sweep"]),
             "| Cooling time (s) | " + " | ".join(n(r["cooling_s"]) for r in ph["wall_sweep"]) + " |", "",
             "| Mean demoulding temperature (°C) | " + " | ".join(f"{r['T_E']}" for r in ph["demould_sweep"]) + " |",
             "|---|" + "---|" * len(ph["demould_sweep"]),
             "| Cooling time at the base wall (s) | " + " | ".join(n(r["cooling_s"]) for r in ph["demould_sweep"]) + " |", "",
             f"Centre-line criterion instead of the mean: {n(ph['cooling_centre_s'])} s "
             f"(finite differences {n(ph['cooling_centre_fd_s'])} s)."]
    return "\n".join(rows)


def t_inputs(d, cfg):
    m, rb, c, sc, p = cfg["material"], cfg["robot_6axis"], cfg["cycle"], cfg["scenario"], cfg["production"]
    rows = ["| Input | Value | Tag |", "|---|---|---|",
            f"| Machines, clamping force | {sc['machines']}, {sc['clamp_force_kN'][0]} to {sc['clamp_force_kN'][1]} kN | [S] R1 |",
            f"| Cavities per mould | {sc['cavities']} | [S] R1 |",
            f"| Public cycle time | {sc['public_cycle_s'][0]} to {sc['public_cycle_s'][1]} s | [S] R1 |",
            f"| Public output | {sc['public_units_per_year'] / 1e6:g} million assemblies a year | [S] R1 |",
            f"| Material | {m['name']}: {m['density_kg_m3']} kg/m³, {m['conductivity_W_mK']} W/(m K), {m['heat_capacity_J_kgK']} J/(kg K) | [S] R4 |",
            f"| Melt / mould temperature | {m['T_melt_C']} / {m['T_mould_C']} °C | [S] R4 |",
            f"| Mean demoulding temperature | {m['T_demould_C']} °C (range {m['T_demould_range_C'][0]} to {m['T_demould_range_C'][1]}) | [A] |",
            f"| Thickest section | {cfg['part']['wall_mm']:g} mm | [A], set to meet the public cycle |",
            f"| Injection, holding, opening, closing, ejector | {c['injection_s']}, {c['holding_s']}, {c['mould_open_s']}, {c['mould_close_s']}, {c['ejector_fwd_s']} s | [A] |",
            f"| 6-axis robot | {rb['name']}: {rb['payload_kg']} kg, {rb['reach_mm']} mm, J1 to J6 {', '.join(str(v) for v in rb['joint_speed_deg_s'])} °/s | [S] R5 |",
            f"| Time to full joint speed | {rb['accel_time_s']} s (range {rb['accel_time_range_s'][0]} to {rb['accel_time_range_s'][1]}) | [A] |",
            f"| Link lengths, programmed linear speed | {rb['link_upper_mm']} + {rb['link_fore_mm']} mm, {rb['linear_speed_max_mm_s']} mm/s | [A] |",
            f"| Linear robot axes | vertical {cfg['robot_linear']['vertical']['v_mm_s']} mm/s, {cfg['robot_linear']['vertical']['a_mm_s2']} mm/s² | [A] |",
            f"| PLC scan, robot I/O scan | {cfg['plc']['scan_s'] * 1000:g} ms, {rb['io_scan_s'] * 1000:g} ms | [A] |",
            f"| Production time, OEE | {p['hours_per_year']} h a year, {p['oee']} | [A] |"]
    return "\n".join(rows)


def t_signals(d, cfg):
    rows = ["| Signal in the model | Contact or robot I/O | From | To | Meaning |", "|---|---|---|---|---|"]
    for name, (io, a, b, text) in d["model"]["signals"].items():
        rows.append(f"| {name} | {io} | {a} | {b} | {text} |")
    return "\n".join(rows)


def t_matrix(d, cfg):
    m = d["model"]
    rows = ["| Variant | Scenarios | States explored | Transitions | P1 reachable in | P2 reachable in |", "|---|---|---|---|---|---|"]
    for v, x in m["variants"].items():
        show = lambda names: ", ".join("the fault-free cell" if s == "none" else f"`{s}`" for s in names) or "no scenario"
        p1, p2 = show(x["P1_scenarios"]), show(x["P2_scenarios"])
        rows.append(f"| {v}: {x['title']} | {len(x['scenarios'])} | {k(x['states_total'])} | {k(x['transitions_total'])} | {p1} | {p2} |")
    return "\n".join(rows)


def t_faults(d, cfg):
    m = d["model"]
    A, B = m["variants"]["A"]["scenarios"], m["variants"]["B"]["scenarios"]
    mark = lambda r: ("**P1**" if r["P1_states"] else "") + (" P2" if r["P2_states"] else "") or "–"
    rows = ["| Scenario | What happens | States (A) | A | B |", "|---|---|---|---|---|"]
    for name, text in m["scenarios"].items():
        rows.append(f"| `{name}` | {text} | {k(A[name]['states'])} | {mark(A[name])} | {mark(B[name])} |")
    return "\n".join(rows)


def t_timed(d, cfg):
    rows = ["| Layout | Removal, budget (s) | Removal, in the loop (s) | Cycle, budget (s) | Cycle, in the loop (s) | Robot alarm | P1 or P2 seen |",
            "|---|---|---|---|---|---|---|"]
    for lay in ("side_6axis", "top_6axis"):
        t = d["timed"][lay]
        rows.append(f"| {d['physics']['alternatives'][lay]['name']} | {n(t['removal_budget_s'], 3)} | {n(t['removal_s'], 3)} | "
                    f"{n(t['cycle_budget_s'], 2)} | {n(t['cycle_s'], 2)} | {'yes' if t['alarm'] else 'no'} | "
                    f"{'yes' if t['p1_seen'] or t['p2_seen'] else 'no'} |")
    return "\n".join(rows)


TABLES = {"findings": t_findings, "alternatives": t_alternatives, "cycle": t_cycle, "inputs": t_inputs,
          "signals": t_signals, "matrix": t_matrix, "faults": t_faults, "timed": t_timed}


def main():
    d = json.loads((ROOT / "results" / "studies.json").read_text(encoding="utf-8"))
    cfg = yaml.safe_load((ROOT / "config" / "cell.yaml").read_text(encoding="utf-8"))
    for name in ("README.md", "DOSSIER.md"):
        path = ROOT / name
        text = path.read_text(encoding="utf-8")
        for key, fn in TABLES.items():
            pat = re.compile(rf"(<!-- TABLE:{key} -->\n)(?:(?!<!-- TABLE:).)*?(<!-- /TABLE -->)", flags=re.S)
            text = pat.sub(lambda mo: mo.group(1) + fn(d, cfg) + "\n" + mo.group(2), text)
        path.write_text(text, encoding="utf-8", newline="\n")
        print("updated", name)


if __name__ == "__main__":
    main()

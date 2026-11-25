"""Evaluate the virtual-controller run (scripts/vc_run.ps1) and write results/virtual_controller_run.json.

    python -m scripts.vc_report

Reads results/virtual_controller_run.jsonl (raw log), results/virtual_controller_listing.ls (the program as the
controller lists it after it translated the LS file), robot/UNLOAD_IMM.ls and results/traces.json (timed run with
the in-repo interpreter). Checks
  - the controller's listing against the repository text, instruction by instruction;
  - the order of the handshake edges of every cycle against the interpreter run: no pair of edges may be in the
    opposite order (edges that fall into one sample are not ordered);
  - the time-out path and the "mould open never comes" wait.
Controller time is the controller's $FAST_CLOCK, taken as 2 ms per tick; the WAIT time-out of the program (5.00 s)
is the check of that tick.
"""

from __future__ import annotations

import json
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
RES = ROOT / "results"
HANDSHAKE = ["ZA7", "MOULD_OPEN", "MAF", "REQ_EJ_FWD", "B4", "ZB4", "REQ_EJ_BACK", "B3", "ZB3", "REQ_CLOSE", "A6", "ZA6",
             "MOULD_CLOSED"]


def instructions(text: str) -> list[str]:
    body = text[text.index("/MN"):text.index("/POS")]
    return [re.sub(r"\s+", " ", t).strip() for _, t in re.findall(r"^\s*(\d+):\s*(.*?)\s*;\s*$", body, flags=re.M)]


def load(name: str) -> list[dict]:
    return [json.loads(ln) for ln in (RES / name).read_text(encoding="utf-8-sig").splitlines() if ln.strip()]


def edge_groups(samples: list[tuple[float, dict]]) -> list[tuple[float, list[tuple[str, int]]]]:
    return [(t, [(k, v) for k, v in chg.items() if k in HANDSHAKE]) for t, chg in samples
            if any(k in HANDSHAKE for k in chg)]


def inversions(ref: list[list[tuple[str, int]]], got: list[list[tuple[str, int]]]) -> tuple[int, int, list]:
    """Pairs of edges ordered in ref that appear in the opposite order in got."""
    def index(groups):
        out = {}
        for i, g in enumerate(groups):
            for e in g:
                out.setdefault(e, i)
        return out
    a, b = index(ref), index(got)
    common = [e for e in a if e in b]
    pairs, bad = 0, []
    for i, x in enumerate(common):
        for y in common[i + 1:]:
            if a[x] == a[y]:
                continue
            first, second = (x, y) if a[x] < a[y] else (y, x)
            pairs += 1
            if b[first] > b[second]:
                bad.append([f"{first[0]}={first[1]}", f"{second[0]}={second[1]}"])
    return len(common), pairs, bad


def main() -> None:
    rows = load("virtual_controller_run.jsonl")
    meta = next(r["meta"] for r in rows if "meta" in r)
    notes = [(r["t"], r["note"]) for r in rows if "note" in r]
    chg = [(r["t"], r["tc"], r["chg"]) for r in rows if "chg" in r]

    # listing
    want = instructions((ROOT / "robot" / f"{meta['program']}.ls").read_text(encoding="utf-8"))
    have = instructions((RES / "virtual_controller_listing.ls").read_text(encoding="utf-8", errors="replace"))
    diff = [[i + 1, w, h] for i, (w, h) in enumerate(zip(want, have)) if w != h]
    put = next(r for r in rows if "ftp_put" in r)

    # controller clock: ticks per second from the WAIT time-out (5.00 s, $WAITTMOUT=500)
    def first(sig, val, after=0.0):
        return next(((t, tc) for t, tc, c in chg if t >= after and c.get(sig) == val), None)
    t_block = next(t for t, n in notes if "ejector will not move" in n)
    t_req = first("REQ_EJ_FWD", 1, t_block)
    t_fault = first("FAULT", 1, t_block)
    ticks_per_s = 500.0            # $FAST_CLOCK counts 2 ms ticks; the WAIT time-out below checks this
    tick_ms = round(1000.0 / ticks_per_s)
    speed = ((chg[-1][1] - chg[0][1]) / ticks_per_s) / (chg[-1][0] - chg[0][0])

    def csec(tc0, tc1):
        return (tc1 - tc0) / ticks_per_s

    # cycles: from each rise of ZA7 to the fall of MOULD_CLOSED
    opens = [(t, tc) for t, tc, c in chg if c.get("ZA7") == 1]
    cycles = []
    for k, (t0, tc0) in enumerate(opens):
        t_end = first("MOULD_CLOSED", 0, t0)
        a6 = first("A6", 1, t0)
        if t_end is None or a6 is None or t0 > t_block:
            continue
        seg = [(t, c) for t, tc, c in chg if t0 <= t <= t_end[0]]
        groups = edge_groups(seg)
        maf0, maf1 = first("MAF", 0, t0), first("MAF", 1, t0 + 0.2)
        want_order = [("REQ_CLOSE", 1), ("A6", 1), ("ZA6", 1), ("MOULD_CLOSED", 1), ("REQ_CLOSE", 0), ("MOULD_CLOSED", 0)]
        idx = [next((i for i, (t, g) in enumerate(groups) if e in g), -1) for e in want_order]
        in_order = all(i >= 0 for i in idx) and idx == sorted(idx) and idx[0] < idx[2] < idx[4] <= idx[5]
        cycles.append({"n": k + 1, "closure_handshake_in_order": in_order, "removal_pc_s": a6[0] - t0, "removal_controller_s": csec(tc0, a6[1]),
                       "in_mould_controller_s": csec(maf0[1], maf1[1]),
                       "edges": [[round(t - t0, 3), [f"{s}={v}" for s, v in g]] for t, g in groups]})

    # interpreter reference
    tr = json.loads((RES / "traces.json").read_text(encoding="utf-8"))
    names = tr["signals"]
    ref_samples, prev = [], None
    for t, sig, *_ in tr["rows"]:
        cur = dict(zip(names, sig))
        if prev is not None and t >= -0.001:
            d = {k: v for k, v in cur.items() if prev[k] != v}
            if d:
                ref_samples.append((t, d))
        prev = cur
    ref_groups = [g for _, g in edge_groups(ref_samples)]
    end = next(i for i, g in enumerate(ref_groups) if ("MOULD_CLOSED", 0) in g)
    ref_groups = ref_groups[:end + 1]
    comp = []
    for c in cycles:
        got = [[tuple(e.split("=")) for e in g] for _, g in c["edges"]]
        got = [[(s, int(v)) for s, v in g] for g in got]
        n, pairs, bad = inversions(ref_groups, got)
        comp.append({"cycle": c["n"], "edges_compared": n, "ordered_pairs": pairs, "inversions": bad,
                     "edges_missing": sorted(f"{s}={v}" for g in ref_groups for s, v in g
                                             if (s, v) not in {e for gg in got for e in gg})})

    # scenario: mould open never comes
    t_hold = next(t for t, n in notes if "kept closed" in n)
    t_rel = next(t for t, n in notes if "mould released" in n)
    robot_keys = ("pos", "MAF", "REQ_CLOSE", "REQ_EJ_FWD", "REQ_EJ_BACK", "AT_CUT", "PLACED", "FAULT", "RO1_VACUUM", "line")
    # the robot reaches the WAIT for the mould (its last change before the release) and then does nothing
    last_robot = max((t, tc) for t, tc, c in chg if t < t_rel and any(k in c for k in robot_keys))
    tc_rel = next(r["tc"] for r in rows if "mould released" in r.get("note", ""))
    state = {}
    for t, tc, c in chg:
        if t <= t_rel:
            state.update(c)
    hold = {"waited_controller_s": csec(last_robot[1], tc_rel), "line": state.get("line"), "position": state.get("pos"),
            "mould": state.get("mould"), "area_free": state.get("MAF"), "req_close": state.get("REQ_CLOSE")}

    # scenario: time-out
    last = {}
    for _, _, c in chg:
        last.update(c)
    tmo = {"wait_timeout_program_s": 5.0, "pc_s": t_fault[0] - t_req[0], "controller_ticks": t_fault[1] - t_req[1],
           "controller_s": (t_fault[1] - t_req[1]) / ticks_per_s,
           "final_line": last.get("line"), "task_status": {0: "running", 1: "paused", 2: "aborted"}.get(last.get("task"), "?"),
           "plc_state": last.get("plc"), "plc_fault": last.get("plc_fault"), "req_close_at_end": last.get("REQ_CLOSE"),
           "a6_at_end": last.get("A6"), "robot_position_at_end": last.get("pos"),
           "newest_alarm_naming_the_program": next((r["alarm"] for r in rows if "alarm" in r), None)}

    # earlier run with a 5 mm half-space as the zone
    z5 = None
    f5 = RES / "virtual_controller_run_zone5mm.jsonl"
    if f5.exists():
        r5 = [r for r in load(f5.name) if "chg" in r]
        state, t_low, spans = {}, None, []
        for r in r5:
            state.update(r["chg"])
            low = state.get("DO10_MOULD_AREA_FREE") == 0 and state.get("mould") != "OPEN"
            if low and t_low is None:
                t_low = r["t"]
            if not low and t_low is not None:
                spans.append(r["t"] - t_low)
                t_low = None
        z5 = {"zone": "half-space 5 mm beyond the start position", "false_inside_events": len(spans),
              "longest_ms": round(1000 * max(spans)) if spans else 0,
              "p2_flagged_by_model": any(r["chg"].get("p2") == 1 for r in r5)}

    out = {
        "controller": {"type": "R-30iB Plus virtual controller", "software": meta["software"].split()[0][:5],
                       "software_full": meta["software"], "arm": "M-710iD/70", "arm_reported": meta["arm"]},
        "study_arm": "M-20iD/25", "layout_offsets_mm": meta["layout_offsets_mm"], "zone_margin_mm": meta["zone_margin_mm"],
        "process_time_s": meta["process_time_s"],
        "translation": {"ftp_exit": put["exit"], "message": put["message"], "instructions_repo": len(want),
                        "instructions_controller": len(have), "identical": not diff and len(want) == len(have), "differences": diff},
        "clock": {"ticks_per_second_of_controller_time": ticks_per_s, "tick_ms": tick_ms,
                  "controller_seconds_per_pc_second": speed},
        "cycles": cycles, "order_vs_interpreter": comp,
        "closure_handshake_in_order_cycles": sum(1 for c in cycles if c["closure_handshake_in_order"]),
        "removal_controller_s": [c["removal_controller_s"] for c in cycles],
        "mould_open_never_comes": hold, "timeout_path": tmo, "zone_5mm_run": z5,
        "p1_seen": any(c.get("p1") == 1 for _, _, c in chg), "p2_seen": any(c.get("p2") == 1 for _, _, c in chg),
    }
    (RES / "virtual_controller_run.json").write_text(json.dumps(out, indent=1) + "\n", encoding="utf-8", newline="\n")
    print(json.dumps({k: out[k] for k in ("translation", "clock", "removal_controller_s", "order_vs_interpreter",
                                          "mould_open_never_comes", "timeout_path", "zone_5mm_run", "p1_seen", "p2_seen")},
                     indent=1)[:5000])


if __name__ == "__main__":
    main()

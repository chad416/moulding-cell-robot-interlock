"""All figures (PNG, docs/img) from results/studies.json and results/traces.json."""
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
IMG = ROOT / "docs" / "img"
IMG.mkdir(parents=True, exist_ok=True)
C = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300", "#6250d6", "#e34948"]
INK, INK2, GRID, SURF = "#0b0b0b", "#52514e", "#e4e3df", "#fcfcfb"
LIMIT = "#e34948"
plt.rcParams.update({"font.size": 9, "axes.edgecolor": INK2, "axes.labelcolor": INK, "xtick.color": INK2,
                     "ytick.color": INK2, "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.6,
                     "axes.spines.top": False, "axes.spines.right": False, "lines.linewidth": 1.6,
                     "figure.facecolor": SURF, "axes.facecolor": SURF, "legend.frameon": False})
ORDER = ["top_linear", "side_6axis", "top_6axis"]


def save(fig, name):
    fig.tight_layout()
    fig.savefig(IMG / name, dpi=150)
    plt.close(fig)


def fig_cycle(d):
    ph = d["physics"]
    alts = ph["alternatives"]
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(10.2, 3.6), gridspec_kw={"width_ratios": [1.5, 1]})
    names = list(alts["side_6axis"]["phases"])
    for i, k in enumerate(ORDER):
        left = 0.0
        for j, n in enumerate(names):
            v = alts[k]["phases"][n]
            a1.barh(i, v, left=left, color=C[j % len(C)], edgecolor=SURF, label=n if i == 0 else None)
            left += v
        a1.text(left + 0.4, i, f"{left:.1f} s", va="center", color=INK)
    a1.set_yticks(range(len(ORDER)), [alts[k]["name"] for k in ORDER])
    a1.invert_yaxis()
    a1.set_xlabel("cycle time (s)")
    a1.set_xlim(0, alts["top_6axis"]["cycle_s"] * 1.12)
    a1.legend(ncol=4, fontsize=7.5, loc="lower center", bbox_to_anchor=(0.5, 1.0))
    a1.grid(axis="y", visible=False)
    w = ph["sensitivity"]["wall"]
    a2.plot([r["wall_mm"] for r in w], [r["cycle_6axis_s"] for r in w], marker="o", ms=4, color=C[0],
            label="cycle, 6-axis side entry")
    a2.axhspan(45, 50, color=C[3], alpha=0.25, lw=0, label="public cycle, 45 to 50 s")
    a2.set_xlabel("wall thickness of the thickest section (mm)")
    a2.set_ylabel("cycle time (s)")
    a2.legend(fontsize=8, loc="upper left")
    save(fig, "cycle_breakdown.png")


def fig_removal(d):
    ph = d["physics"]
    alts = ph["alternatives"]
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(10.2, 3.4), gridspec_kw={"width_ratios": [1.4, 1]})
    parts = [("latency_s", "signals"), ("t_in_s", "move in"), ("t_grip_eject_s", "grip and ejector"),
             ("t_out_s", "move out")]
    for i, k in enumerate(ORDER):
        left = 0.0
        for j, (key, lab) in enumerate(parts):
            v = alts[k][key]
            a1.barh(i, v, left=left, color=C[j], edgecolor=SURF, label=lab if i == 0 else None)
            left += v
        a1.text(left + 0.09, i, f"{left:.2f} s", va="center", color=INK)
    for lay, mark in (("side_6axis", 1), ("top_6axis", 2)):
        a1.plot([d["timed"][lay]["removal_s"]], [mark], marker="D", color=INK, ms=5, ls="none",
                label="in-the-loop run" if mark == 1 else None)
    a1.set_yticks(range(len(ORDER)), [alts[k]["name"] for k in ORDER])
    a1.invert_yaxis()
    a1.set_xlim(0, 2.75)
    a1.set_xlabel('time from "Mould open position" to "Enable mould closure" (s)')
    a1.legend(ncol=5, fontsize=7.5, loc="lower center", bbox_to_anchor=(0.5, 1.0))
    a1.grid(axis="y", visible=False)
    acc = ph["sensitivity"]["accel"]
    a2.plot([r["accel_time_s"] for r in acc], [r["removal_s"] for r in acc], marker="o", ms=4, color=C[1],
            label="6-axis side entry")
    a2.axhline(alts["top_linear"]["removal_s"], color=C[0], ls="--", label="3-axis linear (assumed axes)")
    a2.set_xlabel("time to reach full joint speed (s), assumed")
    a2.set_ylabel("removal time (s)")
    a2.legend(fontsize=8)
    save(fig, "removal_time.png")


def fig_handshake(tr):
    sig = tr["signals"]
    rows = tr["rows"]
    show = [("ZA7", "ZA7 Mould open position"), ("MOULD_OPEN", "DI[2] MOULD_OPEN"),
            ("MAF", "A3/C3 Mould area free"), ("B4", "B4 Enable ejector forward"),
            ("ZB4", "ZB4 Ejector forward position"), ("B3", "B3 Enable ejector back"),
            ("ZB3", "ZB3 Ejector back position"), ("REQ_CLOSE", "DO[1] REQ_CLOSE"),
            ("A6", "A6 Enable mould closure"), ("ZA6", "ZA6 Mould closed"), ("MOULD_CLOSED", "DI[3] MOULD_CLOSED")]
    t_end = 8.5
    fig, ax = plt.subplots(figsize=(10.2, 5.0))
    for i, (key, lab) in enumerate(show):
        k = sig.index(key)
        ts = [r[0] for r in rows] + [t_end]
        vs = [r[1][k] for r in rows]
        vs = vs + [vs[-1]]
        y0 = (len(show) - 1 - i) * 1.4
        col = LIMIT if key in ("MAF", "A6") else (C[0] if key.startswith("Z") else (C[2] if key.startswith("B") else INK2))
        ax.step(ts, [y0 + 0.9 * v for v in vs], where="post", color=col, lw=1.5)
        ax.text(-1.15, y0 + 0.3, lab, ha="right", va="center", fontsize=8, color=INK)
    inside = [r[0] for r in rows if "MOULD_IN" in r[3] or "GRIP" in r[3]]
    if inside:
        t_in = min(t for t in inside)
        t_out = next(r[0] for r in rows if r[0] > t_in and r[1][sig.index("MAF")] == 1)
        ax.axvspan(t_in, t_out, color=C[3], alpha=0.18, lw=0)
        ax.text((t_in + t_out) / 2, len(show) * 1.4 - 0.2, "gripper in the mould area", ha="center", fontsize=8, color=INK2)
    ax.set_xlim(-1.0, t_end)
    ax.set_ylim(-0.4, len(show) * 1.4 + 0.3)
    ax.set_yticks([])
    ax.set_xlabel('time after "Mould open position" (s), in-the-loop run, 6-axis side entry')
    ax.grid(axis="y", visible=False)
    ax.spines["left"].set_visible(False)
    save(fig, "handshake_timing.png")


def fig_state_space(d):
    m = d["model"]
    fig, axes = plt.subplots(1, 2, figsize=(10.2, 7.6), sharey=True)
    names = list(m["variants"]["A"]["scenarios"])
    for ax, v in zip(axes, ("A", "B")):
        rows = m["variants"][v]["scenarios"]
        cols = [LIMIT if rows[n]["P1_states"] else (C[3] if rows[n]["P2_states"] else C[0]) for n in names]
        ax.barh(range(len(names)), [rows[n]["states"] / 1000 for n in names], color=cols)
        ax.set_yticks(range(len(names)), names, fontsize=6.3)
        ax.set_xlabel("reachable states (thousands), all explored")
        ax.set_title(f"variant {v}: " + ("contact A3/C3 used" if v == "A" else "contact A3/C3 bridged"), fontsize=9, loc="left")
        ax.grid(axis="y", visible=False)
    axes[0].invert_yaxis()
    from matplotlib.patches import Patch
    fig.legend(handles=[Patch(color=C[0], label="P1 and P2 unreachable"),
                        Patch(color=C[3], label="P2 reachable (gripper in a mould that is not open)"),
                        Patch(color=LIMIT, label="P1 reachable (mould closes on the gripper)")],
               fontsize=8, loc="upper center", ncol=3)
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    fig.savefig(IMG / "state_space.png", dpi=150)
    plt.close(fig)


def fig_counterexample(d):
    tr = d["model"]["counterexamples"]["C/none/P1"]
    tail = tr[-16:]
    n0 = len(tr) - len(tail)
    rows = [("inside", "gripper in the mould area", C[3]), ("REQ_CLOSE", "DO[1] REQ_CLOSE (wired to A6)", INK2),
            ("A6", "A6 Enable mould closure", LIMIT), ("closing", "mould closing movement", LIMIT)]
    fig, ax = plt.subplots(figsize=(10.2, 4.3))
    xs = list(range(len(tail) + 1))
    for i, (key, lab, col) in enumerate(rows):
        y0 = (len(rows) - 1 - i) * 1.4
        vs = [int(r[key]) for r in tail]
        ax.step(xs, [y0 + 0.9 * v for v in vs + [vs[-1]]], where="post", color=col, lw=1.6)
        ax.text(-0.3, y0 + 0.3, lab, ha="right", va="center", fontsize=8)
    for j, r in enumerate(tail):
        ax.text(j + 0.5, -1.0, f"{n0 + j}: {r['step']}\n{r['robot']}", rotation=60, ha="right", va="top", fontsize=6.5,
                color=INK2)
    ax.axvspan(len(tail) - 1, len(tail), color=LIMIT, alpha=0.15, lw=0)
    ax.text(len(tail) - 0.5, len(rows) * 1.4 - 0.1, "P1", ha="center", color=LIMIT, fontsize=10, fontweight="bold")
    ax.set_xlim(0, len(tail))
    ax.set_ylim(-0.6, len(rows) * 1.4 + 0.3)
    ax.set_yticks([])
    ax.set_xticks([])
    ax.grid(visible=False)
    ax.spines["left"].set_visible(False)
    ax.set_title(f"Weakened handshake (variant C), no fault: shortest path to the collision has {len(tr) - 1} steps; "
                 f"the last {len(tail)} are shown", fontsize=9, loc="left")
    fig.subplots_adjust(bottom=0.36, left=0.27, right=0.98, top=0.90)
    fig.savefig(IMG / "counterexample.png", dpi=150)
    plt.close(fig)


def fig_vc(d):
    """Handshake of one cycle on the virtual controller (results/virtual_controller_run.json)."""
    cyc = d["cycles"][min(2, len(d["cycles"]) - 1)]
    show = [("ZA7", "ZA7 Mould open position"), ("MOULD_OPEN", "DI[2] MOULD_OPEN"),
            ("MAF", "DO[10] / A3-C3 Mould area free"), ("B4", "B4 Enable ejector forward"),
            ("ZB4", "ZB4 Ejector forward position"), ("B3", "B3 Enable ejector back"),
            ("ZB3", "ZB3 Ejector back position"), ("REQ_CLOSE", "DO[1] REQ_CLOSE"),
            ("A6", "A6 Enable mould closure"), ("ZA6", "ZA6 Mould closed"), ("MOULD_CLOSED", "DI[3] MOULD_CLOSED")]
    start = {"ZA7": 0, "MOULD_OPEN": 0, "MAF": 1, "B4": 0, "ZB4": 0, "B3": 0, "ZB3": 1, "REQ_CLOSE": 0, "A6": 0,
             "ZA6": 0, "MOULD_CLOSED": 0}
    t_end = max(t for t, _ in cyc["edges"]) + 0.6
    fig, ax = plt.subplots(figsize=(10.2, 5.0))
    for i, (key, lab) in enumerate(show):
        ts, vs, v = [-0.6], [start[key]], start[key]
        for t, edges in cyc["edges"]:
            for e in edges:
                name, val = e.split("=")
                if name == key:
                    ts.append(t)
                    v = int(val)
                    vs.append(v)
        ts.append(t_end)
        vs.append(v)
        y0 = (len(show) - 1 - i) * 1.4
        col = LIMIT if key in ("MAF", "A6") else (C[0] if key.startswith("Z") else (C[2] if key.startswith("B") else INK2))
        ax.step(ts, [y0 + 0.9 * x for x in vs], where="post", color=col, lw=1.5)
        ax.text(-0.75, y0 + 0.3, lab, ha="right", va="center", fontsize=8, color=INK)
    ax.set_xlim(-0.6, t_end)
    ax.set_ylim(-0.4, len(show) * 1.4 + 0.3)
    ax.set_yticks([])
    c = d["controller"]
    ax.set_xlabel('time after "Mould open position" (s, PC clock)')
    ax.set_title(f'TP program on the {c["type"]}, software {c["software"]}, arm {c["arm"]}; machine model and PLC block in the loop',
                 fontsize=9, loc="left")
    ax.grid(axis="y", visible=False)
    ax.spines["left"].set_visible(False)
    save(fig, "vc_handshake_timing.png")


def main():
    d = json.loads((ROOT / "results" / "studies.json").read_text(encoding="utf-8"))
    tr = json.loads((ROOT / "results" / "traces.json").read_text(encoding="utf-8"))
    fig_cycle(d)
    fig_removal(d)
    fig_handshake(tr)
    fig_state_space(d)
    fig_counterexample(d)
    vc = ROOT / "results" / "virtual_controller_run.json"
    if vc.exists():
        fig_vc(json.loads(vc.read_text(encoding="utf-8")))
    print("figures written to", IMG)


if __name__ == "__main__":
    main()

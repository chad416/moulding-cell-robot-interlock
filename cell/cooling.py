"""Cooling time of a plate-shaped section and the cycle breakdown.

Plate of thickness s, uniform melt temperature T_M at t = 0, both faces held at the mould wall temperature T_W,
constant effective thermal diffusivity a. The first term of the Fourier series gives the textbook relations
(Menges / Mohren; the centre-line form is printed in R3):

    centre line reaches T_E:   t = s^2 / (pi^2 a) * ln( 4/pi   * (T_M - T_W) / (T_E - T_W) )
    mean temperature at T_E:   t = s^2 / (pi^2 a) * ln( 8/pi^2 * (T_M - T_W) / (T_E - T_W) )

`plate_fd` solves the same problem with finite differences; the tests compare both.
"""

from __future__ import annotations

import math


def diffusivity_mm2_s(conductivity_W_mK: float, density_kg_m3: float, heat_capacity_J_kgK: float) -> float:
    return conductivity_W_mK / (density_kg_m3 * heat_capacity_J_kgK) * 1e6


def cooling_time(s_mm: float, a_mm2_s: float, T_M: float, T_W: float, T_E: float, criterion: str = "mean") -> float:
    k = {"mean": 8.0 / math.pi ** 2, "centre": 4.0 / math.pi}[criterion]
    theta = (T_M - T_W) / (T_E - T_W)
    return s_mm ** 2 / (math.pi ** 2 * a_mm2_s) * math.log(k * theta)


def plate_fd(s_mm: float, a_mm2_s: float, T_M: float, T_W: float, T_E: float, criterion: str = "mean",
             n: int = 81) -> float:
    """Explicit finite differences over the full thickness; returns the time at which the criterion is met."""
    dx = s_mm / (n - 1)
    dt = 0.4 * dx * dx / a_mm2_s
    T = [T_M] * n
    T[0] = T[-1] = T_W
    t = 0.0
    r = a_mm2_s * dt / (dx * dx)
    while True:
        if criterion == "centre":
            val = T[n // 2]
        else:  # trapezoidal mean over the thickness
            val = (sum(T) - 0.5 * (T[0] + T[-1])) / (n - 1)
        if val <= T_E:
            return t
        T = [T_W] + [T[i] + r * (T[i - 1] - 2 * T[i] + T[i + 1]) for i in range(1, n - 1)] + [T_W]
        t += dt


def cycle_breakdown(cfg: dict, removal_s: float, wall_mm: float | None = None, T_E: float | None = None) -> dict:
    """Cycle phases in seconds. The holding time runs inside the cooling time (the part cools from the end of
    injection); the mould stands open for `removal_s` between "Mould open position" and "Enable mould closure"
    plus the closing permission delay, which is part of removal_s."""
    m, c = cfg["material"], cfg["cycle"]
    a = diffusivity_mm2_s(m["conductivity_W_mK"], m["density_kg_m3"], m["heat_capacity_J_kgK"])
    s = cfg["part"]["wall_mm"] if wall_mm is None else wall_mm
    te = m["T_demould_C"] if T_E is None else T_E
    t_cool = cooling_time(s, a, m["T_melt_C"], m["T_mould_C"], te, "mean")
    hold = min(c["holding_s"], t_cool)
    phases = {
        "injection": c["injection_s"],
        "holding": hold,
        "remaining cooling": t_cool - hold,
        "mould opening": c["mould_open_s"],
        "removal (mould open)": removal_s,
        "mould closing": c["mould_close_s"],
        "overhead": c["dry_overhead_s"],
    }
    return {"a_mm2_s": a, "wall_mm": s, "T_demould_C": te, "cooling_s": t_cool, "phases": phases,
            "cycle_s": sum(phases.values())}


def annual_output(cfg: dict, cycle_s: float) -> dict:
    p, sc = cfg["production"], cfg["scenario"]
    shots = p["hours_per_year"] * 3600.0 * p["oee"] / cycle_s
    per_machine = shots * sc["cavities"]
    return {"shots_per_machine": shots, "parts_per_machine": per_machine, "parts_plant": per_machine * sc["machines"],
            "parts_per_public_unit": per_machine * sc["machines"] / sc["public_units_per_year"]}

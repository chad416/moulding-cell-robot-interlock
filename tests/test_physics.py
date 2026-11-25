"""Cooling relation against an independent finite-difference solution; move-time model checks."""
import math
from pathlib import Path

import pytest
import yaml

from cell import cooling, motion

ROOT = Path(__file__).resolve().parents[1]
CFG = yaml.safe_load((ROOT / "config" / "cell.yaml").read_text(encoding="utf-8"))


@pytest.mark.parametrize("criterion", ["mean", "centre"])
@pytest.mark.parametrize("wall", [3.0, 7.0])
def test_cooling_relation_matches_finite_differences(criterion, wall):
    args = (wall, 0.1765, 280.0, 80.0, 120.0)
    assert cooling.plate_fd(*args, criterion) == pytest.approx(cooling.cooling_time(*args, criterion), rel=0.01)


def test_cooling_time_scales_with_the_square_of_the_wall():
    t1 = cooling.cooling_time(3.0, 0.15, 280, 80, 120)
    assert cooling.cooling_time(6.0, 0.15, 280, 80, 120) == pytest.approx(4 * t1)


def test_diffusivity_from_the_datasheet_values():
    m = CFG["material"]
    a = cooling.diffusivity_mm2_s(m["conductivity_W_mK"], m["density_kg_m3"], m["heat_capacity_J_kgK"])
    assert a == pytest.approx(0.36 / (1360 * 1500) * 1e6)


def test_cycle_is_the_sum_of_its_phases_and_removal_adds_one_to_one():
    a = cooling.cycle_breakdown(CFG, 2.0)
    b = cooling.cycle_breakdown(CFG, 3.0)
    assert a["cycle_s"] == pytest.approx(sum(a["phases"].values()))
    assert b["cycle_s"] - a["cycle_s"] == pytest.approx(1.0)


def test_trapezoid_short_and_long_moves():
    assert motion.trapezoid(100.0, 1000.0, 10000.0) == pytest.approx(2 * math.sqrt(100.0 / 10000.0))   # triangle
    assert motion.trapezoid(1000.0, 1000.0, 10000.0) == pytest.approx(1.0 + 0.1)                      # cruise
    assert motion.trapezoid(100.0, 1000.0, 10000.0) == pytest.approx(motion.trapezoid(100.0 + 1e-9, 1000.0, 10000.0))


def test_inverse_kinematics_reaches_the_point():
    rb = motion.SixAxis(CFG["robot_6axis"])
    for name, p in CFG["layouts"]["side_6axis"]["positions"].items():
        q1, q2, q3, *_ = rb.ik(p["xyz"])
        r = rb.l1 * math.cos(q2) + rb.l2 * math.cos(q2 - q3)
        z = rb.l1 * math.sin(q2) + rb.l2 * math.sin(q2 - q3)
        assert (r * math.cos(q1), r * math.sin(q1), z) == pytest.approx(tuple(p["xyz"]), abs=1e-6), name


def test_every_program_position_is_inside_the_reach():
    rb = CFG["robot_6axis"]
    for lay in ("side_6axis", "top_6axis"):
        for name, p in CFG["layouts"][lay]["positions"].items():
            assert math.dist(p["xyz"], (0, 0, 0)) < rb["link_upper_mm"] + rb["link_fore_mm"] <= rb["reach_mm"], name


def test_linear_move_respects_the_programmed_speed_and_the_joint_limits():
    rb = motion.SixAxis(CFG["robot_6axis"])
    pos = {k: v["xyz"] for k, v in CFG["layouts"]["side_6axis"]["positions"].items()}
    a, b = pos["WAIT_IMM"], pos["MOULD_IN"]
    t = rb.linear_move(a, b, 2000)
    assert t >= math.dist(a, b) / 2000.0
    qa, qb = rb.ik(a), rb.ik(b)
    assert all(abs(qb[j] - qa[j]) / t <= rb.w[j] for j in range(6))      # mean joint speed below the limit


def test_removal_budget_adds_up_and_six_axis_is_slower_than_linear_here():
    lin, side, top = (motion.removal_budget(CFG, k) for k in ("top_linear", "side_6axis", "top_6axis"))
    for b in (lin, side, top):
        assert b["removal_s"] == pytest.approx(b["latency_s"] + b["t_in_s"] + b["t_grip_eject_s"] + b["t_out_s"])
    assert lin["removal_s"] < side["removal_s"] < top["removal_s"]


def test_annual_output_arithmetic():
    o = cooling.annual_output(CFG, 48.0)
    assert o["parts_per_machine"] == pytest.approx(5760 * 3600 * 0.85 / 48.0 * 4)
    assert o["parts_plant"] == pytest.approx(14 * o["parts_per_machine"])

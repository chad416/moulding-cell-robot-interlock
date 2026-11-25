"""State-space results, liveness, the timed in-the-loop run and its agreement with the model.
These tests need the compiled controller (scripts/build_plc.sh: matiec and gcc, Linux or WSL)."""
from pathlib import Path

import pytest
import yaml

from cell import plc as plcmod
from cell.checker import can_always_reach, explore
from cell.loop import observable, run_timed
from cell.motion import removal_budget
from cell.system import System

ROOT = Path(__file__).resolve().parents[1]
CFG = yaml.safe_load((ROOT / "config" / "cell.yaml").read_text(encoding="utf-8"))
pytestmark = pytest.mark.skipif(not plcmod.available(), reason="build/plc/libplc.so not built")
FAULTS = ["A6_stuck_high", "REQ_CLOSE_stuck_high", "MAF_stuck_high", "AREA_FREE_stuck_high", "zone_function_stuck_free", "ZB3_stuck_high",
          "MOULD_CLOSED_stuck_low", "robot_hold", "gate_forced", "gate_request", "robot_power_loss",
          "plc_power_loss_A6_high", "imm_power_loss"]


@pytest.fixture(scope="module")
def plc():
    return plcmod.Plc()


@pytest.fixture(scope="module")
def base(plc):
    sysm = System(plc, "A", "none")
    return sysm, explore(sysm)


def test_design_without_fault_never_reaches_a_hazard_state(base):
    _, r = base
    assert r["states"] > 5000 and r["violations"] == {"P1": 0, "P2": 0}


def test_a6_is_never_on_while_the_gripper_is_in_the_mould_area(base):
    sysm, r = base
    a6 = sysm.oi["QA6_ENABLEMOULDCLOSURE"]
    assert not any(s.out[a6] and sysm.inside(s.rob) for s in r["parent"])


@pytest.mark.parametrize("fault", FAULTS)
def test_design_keeps_the_mould_off_the_gripper_under_a_single_fault(plc, fault):
    r = explore(System(plc, "A", fault))
    assert r["violations"]["P1"] == 0


def test_known_limit_mould_open_signal_has_one_channel(plc):
    r = explore(System(plc, "A", "ZA7_stuck_high"))
    assert r["violations"]["P1"] == 0 and r["violations"]["P2"] > 0


def test_bridged_contacts_are_safe_without_fault_but_one_stuck_output_closes_the_mould_on_the_gripper(plc):
    assert explore(System(plc, "B", "none"))["violations"]["P1"] == 0
    sysm = System(plc, "B", "A6_stuck_high")
    r = explore(sysm)
    trace = r["trace"]["P1"]
    assert r["violations"]["P1"] > 0 and sysm.hazards(trace[-1][1])[0]
    for (_, a), (lab, b) in zip(trace, trace[1:]):               # the printed trace is a real path of the model
        assert (lab, b) in sysm.transitions(a)


def test_weakened_handshake_collides_without_any_fault_and_the_full_design_absorbs_the_same_program(plc):
    assert explore(System(plc, "C", "none"))["violations"]["P1"] > 0
    assert explore(System(plc, "D", "none"))["violations"] == {"P1": 0, "P2": 0}


def test_handshake_cannot_deadlock(plc):
    sysm = System(plc, "A", "none", timeouts=False, plc_faster_than_process=True)
    r = explore(sysm, keep_edges=True)
    live = can_always_reach(r, "imm:process_done")
    assert live["can_reach"] == live["states"] > 500
    assert all(r["edges"][s] for s in r["edges"])


def test_timed_run_meets_the_budget_and_stays_inside_the_model(plc, base):
    _, r = base
    run = run_timed(plc, CFG, "side_6axis", cycles=3)
    assert not run["alarm"] and not run["p1_seen"] and not run["p2_seen"] and len(run["cycles"]) == 2
    budget = removal_budget(CFG, "side_6axis")["removal_s"]
    for c in run["cycles"]:
        assert c["removal_s"] == pytest.approx(budget, abs=0.03)
    reach = {observable(s) for s in r["parent"]}
    assert {observable(s) for s in run["states"]} <= reach


def test_watchdog_stops_a_closure_that_does_not_finish(plc):
    sysm = System(plc, "A", "none", close_tmo_scans=3)
    names = plc.input_names
    vals = {n: 0 for n in names}
    vals.update(ISAFETYOK=1, IGATECLOSEDLOCKED=1, RBAREAFREE=1, IZB3_EJECTORBACK=1, IZB2_ENABLEOPERATION=1,
                ICUTOPEN=1, INESTFREE=1, CFGCLOSETMOSCANS=3)
    st = plc.initial
    seq = [dict(IZA6_MOULDCLOSED=1), dict(IZA6_MOULDCLOSED=0, IZA7_MOULDOPENPOSITION=1), dict(RBAREAFREE=0),
           dict(RBAREAFREE=1, RBREQCLOSE=1)] + [dict()] * 5
    seen = []
    for step in seq:
        vals.update(step)
        st, out = plc.scan(st, tuple(vals[n] for n in names))
        seen.append((out[sysm.oi["STATE"]], out[sysm.oi["QA6_ENABLEMOULDCLOSURE"]], out[sysm.oi["FAULTCODE"]]))
    c = plc.constants
    assert (c["S_CLOSE"], 1, 0) in seen
    assert seen[-1] == (c["S_FAULT"], 0, c["F_CLOSE_TMO"])

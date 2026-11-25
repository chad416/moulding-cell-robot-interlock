"""The self-checking PLC test bench (plc/test): it passes against the block compiled with matiec, it fails when
an expected value is wrong, and the committed bench files are what the generator produces from the model.
Needs the compiled controller and bench (scripts/build_plc.sh, scripts/build_bench.sh: matiec and gcc)."""
import ctypes
import json
import os
import subprocess
import sys
from pathlib import Path

import pytest

from cell import plc as plcmod

ROOT = Path(__file__).resolve().parents[1]
BENCH = ROOT / "build" / "plc_test"
CASES = json.loads((ROOT / "results" / "testbench_cases.json").read_text(encoding="utf-8"))
MATIEC = Path(os.environ.get("MATIEC_DIR", str(Path.home() / "matiec")))
pytestmark = pytest.mark.skipif(not (plcmod.available() and plcmod.available(BENCH)),
                                reason="build/plc and build/plc_test not built")


def run_bench(build_dir: Path, max_scans: int = 20000) -> tuple[int, dict]:
    p = plcmod.Plc(build_dir)
    inp, out = p.In(), p.Out()
    inp.ENABLE = 1
    n = 0
    while not out.DONE and n < max_scans:
        p.lib.rt_scan(ctypes.byref(inp), ctypes.byref(out))
        n += 1
    result = {k: getattr(out, k) for k in p.output_names}
    for _ in range(50):                                   # the result is held
        p.lib.rt_scan(ctypes.byref(inp), ctypes.byref(out))
    assert result == {k: getattr(out, k) for k in p.output_names}
    return n, result


def test_bench_passes_against_the_matiec_build_and_ends_on_its_own():
    n, r = run_bench(BENCH)
    nc = len(CASES["cases"])
    assert n == CASES["total_scans"]
    assert r == {"DONE": 1, "CASESTOTAL": nc, "CASESRUN": nc, "CASESPASSED": nc, "CASESFAILED": 0,
                 "FIRSTFAILEDCASE": 0, "FIRSTFAILEDCHECK": 0, "CASEPASSBITS": (1 << nc) - 1}


def test_cases_end_where_the_scenario_says():
    by = {c["key"]: c for c in CASES["cases"]}
    assert len(by) == 7
    for key in ("normal_cycles", "robot_late", "access_on_request", "gate_opened_and_reset"):
        assert (by[key]["end_state"], by[key]["end_fault"], by[key]["cycles"]) == ("S_MOULDING", "none", 3), key
    assert (by["area_free_lost_during_closing"]["end_state"], by["area_free_lost_during_closing"]["end_fault"]) == ("S_FAULT", "F_AREA")
    assert (by["conveyor_not_ready"]["end_state"], by["conveyor_not_ready"]["end_fault"]) == ("S_FAULT", "F_ROBOT")
    assert (by["closing_watchdog"]["end_state"], by["closing_watchdog"]["end_fault"]) == ("S_FAULT", "F_CLOSE_TMO")
    assert "S_ACCESS" in by["access_on_request"]["state_sequence"]
    assert by["gate_opened_and_reset"]["state_sequence"].count("S_FAULT") == 1


def test_committed_bench_files_are_what_the_generator_writes(tmp_path):
    subprocess.run([sys.executable, "-m", "scripts.make_testbench", "--out", str(tmp_path)], cwd=ROOT, check=True,
                   capture_output=True)
    for rel in ("plc/test/st/tb_bench.st", "plc/test/scl/TbBench.scl", "plc/test/scl/DB_TestResult.scl",
                "plc/test/scl/Main_Test.scl", "results/testbench_cases.json", "docs/PLCSIM_RUN.md"):
        new = (tmp_path / rel).read_text(encoding="utf-8").replace("\r\n", "\n")
        old = (ROOT / rel).read_text(encoding="utf-8").replace("\r\n", "\n")
        if rel == "docs/PLCSIM_RUN.md":                   # the result section is filled in by hand after the run
            cut = "<!-- PLCSIM-RESULT"
            new, old = new[:new.index(cut)], old[:old.index(cut)]
        assert new == old, rel


@pytest.mark.skipif(not (MATIEC / "iec2c").exists(), reason="matiec not found")
def test_bench_fails_when_one_expected_bit_is_wrong(tmp_path):
    case = CASES["cases"][2]
    subprocess.run([sys.executable, "-m", "scripts.make_testbench", "--out", str(tmp_path), "--break", case["key"]],
                   cwd=ROOT, check=True, capture_output=True)
    out_dir = tmp_path / "build"
    subprocess.run(["bash", "scripts/build_bench.sh", str(tmp_path / "plc/test/st/tb_bench.st"), str(out_dir)],
                   cwd=ROOT, check=True, capture_output=True, env={**os.environ, "PYTHON": sys.executable})
    n, r = run_bench(out_dir)
    nc = len(CASES["cases"])
    assert r["DONE"] == 1 and r["CASESRUN"] == nc and r["CASESPASSED"] == nc - 1 and r["CASESFAILED"] == 1
    assert r["FIRSTFAILEDCASE"] == 3 and r["FIRSTFAILEDCHECK"] == case["scans"] // 2 + 1
    assert r["CASEPASSBITS"] == ((1 << nc) - 1) & ~(1 << 2)
    assert n < CASES["total_scans"]

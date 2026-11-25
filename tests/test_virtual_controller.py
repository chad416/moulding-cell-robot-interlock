"""Records of the run on the FANUC virtual controller (scripts/vc_run.ps1). The run itself cannot be repeated in
CI; these tests keep the committed records, their evaluation and the program text consistent."""
import json
import re
from pathlib import Path

from scripts import vc_report

ROOT = Path(__file__).resolve().parents[1]
RES = ROOT / "results"
V = json.loads((RES / "virtual_controller_run.json").read_text(encoding="utf-8"))


def test_the_controller_listing_is_the_repository_program():
    want = vc_report.instructions((ROOT / "robot" / "UNLOAD_IMM.ls").read_text(encoding="utf-8"))
    have = vc_report.instructions((RES / "virtual_controller_listing.ls").read_text(encoding="utf-8"))
    assert len(want) == 72 and have == want
    assert V["translation"]["identical"] and V["translation"]["ftp_exit"] == 0


def test_tp_comments_fit_the_controller_limit():
    for path in (ROOT / "robot" / "UNLOAD_IMM.ls", ROOT / "robot" / "variants" / "UNLOAD_EARLY.ls"):
        for text in re.findall(r"^\s*\d+:\s*!(.*?)\s*;\s*$", path.read_text(encoding="utf-8"), flags=re.M):
            assert len(text) <= 32, text


def test_evaluation_is_reproducible_from_the_raw_log():
    before = (RES / "virtual_controller_run.json").read_text(encoding="utf-8")
    vc_report.main()
    assert (RES / "virtual_controller_run.json").read_text(encoding="utf-8") == before


def test_handshake_order_matches_the_interpreter_run():
    assert len(V["cycles"]) == 4 and V["closure_handshake_in_order_cycles"] == 4
    for c in V["order_vs_interpreter"]:
        assert c["edges_compared"] == 24 and c["inversions"] == [] and c["edges_missing"] == []
    assert not V["p1_seen"] and not V["p2_seen"]


def test_wait_without_timeout_and_the_timeout_path():
    hold, tmo = V["mould_open_never_comes"], V["timeout_path"]
    assert hold["waited_controller_s"] > 10 and hold["position"] == "WAIT_IMM" and hold["req_close"] == 0
    assert abs(tmo["controller_s"] - 5.0) < 0.2
    assert tmo["task_status"] == "paused" and tmo["a6_at_end"] == 0 and tmo["req_close_at_end"] == 0
    assert tmo["plc_fault"] == "F_ROBOT" and "INTP-213" in tmo["newest_alarm_naming_the_program"]


def test_records_name_only_the_controller_type_software_and_arm():
    c = V["controller"]
    assert (c["type"], c["software"], c["arm"]) == ("R-30iB Plus virtual controller", "V9.40", "M-710iD/70")

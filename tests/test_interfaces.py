"""The robot program, the I/O map, the ST controller, the SCL export and the layout agree."""
import re
import sys
from pathlib import Path

import yaml

from cell.tp import Program

ROOT = Path(__file__).resolve().parents[1]
IO = yaml.safe_load((ROOT / "robot" / "io_map.yaml").read_text(encoding="utf-8"))
CFG = yaml.safe_load((ROOT / "config" / "cell.yaml").read_text(encoding="utf-8"))
ST = (ROOT / "plc" / "st" / "cell_plc.st").read_text(encoding="utf-8")
PROGRAMS = [ROOT / "robot" / "UNLOAD_IMM.ls", ROOT / "robot" / "variants" / "UNLOAD_EARLY.ls"]


def st_names(section):
    body = re.search(rf"\b{section}\b(.*?)\bEND_VAR\b", ST, flags=re.S).group(1)
    body = re.sub(r"\(\*.*?\*\)", "", body, flags=re.S)
    return {d.split(":")[0].strip() for d in body.split(";") if ":" in d}


def test_every_robot_signal_in_the_programs_is_in_the_io_map_with_the_same_name():
    kinds = (("robot_di", "DI"), ("robot_do", "DO"), ("robot_zone_do", "DO"), ("robot_ro", "RO"), ("robot_ri", "RI"))
    known = {(k, n, v["name"]) for sec, k in kinds for n, v in IO[sec].items()}
    for path in PROGRAMS:
        used = Program(path).signals
        assert used and used <= known, (path.name, used - known)


def test_io_map_points_at_real_plc_variables():
    ins, outs = st_names("VAR_INPUT"), st_names("VAR_OUTPUT")
    assert {v["plc"] for v in IO["robot_do"].values()} | {v["plc"] for v in IO["robot_zone_do"].values()} <= ins
    assert {v["plc"] for v in IO["robot_di"].values()} <= outs


def test_program_header_line_count_matches_and_positions_exist():
    for path in PROGRAMS:
        p = Program(path)
        assert p.line_count == len(p.lines)
        assert set(p.positions) <= set(IO["position_registers"].items())
    for lay in ("side_6axis", "top_6axis"):
        assert set(IO["position_registers"].values()) == set(CFG["layouts"][lay]["positions"])


def test_the_program_never_writes_the_zone_signal():
    for path in PROGRAMS:
        assert not re.search(r"^\s*\d+:\s*DO\[10", path.read_text(encoding="utf-8"), flags=re.M)


def test_ls_files_match_their_generator():
    sys.path.insert(0, str(ROOT / "tools"))
    import make_ls
    bodies = (make_ls.INIT + make_ls.NORMAL_EXIT + make_ls.REST, make_ls.INIT + make_ls.EARLY_EXIT + make_ls.REST)
    for path, body in zip(PROGRAMS, bodies):
        assert [t for _, t in Program(path).lines] == body.strip().split("\n")


def test_scl_export_is_up_to_date_with_the_st_source():
    sys.path.insert(0, str(ROOT / "plc"))
    import st_to_scl
    have = (ROOT / "plc" / "scl" / "CellPlc.scl").read_text(encoding="utf-8").replace("\r\n", "\n")
    assert st_to_scl.convert(ST).replace("\r\n", "\n") == have


def test_st_block_names_the_euromap_67_contacts_it_maps():
    for contact in ("ZA6", "ZA7", "ZB2", "ZB3", "ZB4", "A6", "B2", "B3", "B4"):
        assert re.search(rf"\b[iq]{contact}_", ST), contact

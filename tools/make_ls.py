"""Write robot/UNLOAD_IMM.ls and the weakened variant with consistent line numbers and LINE_COUNT.

The instruction lists below are the source; run this after editing them:  python tools/make_ls.py
"""
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
HEAD = """/PROG  {name}
/ATTR
OWNER		= MNEDITOR;
COMMENT		= "{comment}";
PROG_SIZE	= 0;
CREATE		= DATE 26-11-25  TIME 12:00:00;
MODIFIED	= DATE 26-11-25  TIME 12:00:00;
FILE_NAME	= ;
VERSION		= 0;
LINE_COUNT	= {n};
MEMORY_SIZE	= 0;
PROTECT		= READ_WRITE;
TCD:  STACK_SIZE	= 0,
      TASK_PRIORITY	= 50,
      TIME_SLICE	= 0,
      BUSY_LAMP_OFF	= 0,
      ABORT_REQUEST	= 0,
      PAUSE_REQUEST	= 0;
DEFAULT_GROUP	= 1,*,*,*,*;
CONTROL_CODE	= 00000000 00000000;
/APPL
/MN
"""
INIT = """!Unload the mould of an injection
!moulding machine. Handshake with
!PLC FB CELL_PLC (EUROMAP 67)
!Signals: robot/io_map.yaml
UFRAME_NUM=1
UTOOL_NUM=1
$WAITTMOUT=500
DO[1:REQ_CLOSE]=OFF
DO[2:REQ_EJ_FWD]=OFF
DO[3:REQ_EJ_BACK]=OFF
DO[4:AT_CUT]=OFF
DO[5:PLACED]=OFF
DO[6:FAULT]=OFF
DO[7:AT_WAIT]=OFF
J PR[2:WAIT_IMM] 100% FINE
LBL[1:CYCLE]
IF DI[8:CYCLE_ENABLE]=ON,JMP LBL[2]
!Access requested: park here
DO[7:AT_WAIT]=ON
WAIT DI[8:CYCLE_ENABLE]=ON
DO[7:AT_WAIT]=OFF
JMP LBL[1]
LBL[2:RUN]
WAIT DI[1:IMM_AUTO]=ON
WAIT (DI[2:MOULD_OPEN]=ON OR DI[8:CYCLE_ENABLE]=OFF)
IF DI[8:CYCLE_ENABLE]=OFF,JMP LBL[1]
!Last check before the move
IF DI[2:MOULD_OPEN]=OFF,JMP LBL[1]
!Enter: DO[10] goes OFF when the
!robot leaves the start position
L PR[3:MOULD_IN] 2000mm/sec FINE
L PR[4:GRIP] 500mm/sec FINE
RO[1:VACUUM]=ON
DO[2:REQ_EJ_FWD]=ON
WAIT DI[4:EJ_FWD]=ON TIMEOUT,LBL[99]
WAIT RI[1:PART_OK]=ON TIMEOUT,LBL[99]
DO[2:REQ_EJ_FWD]=OFF
L PR[3:MOULD_IN] 500mm/sec FINE
DO[3:REQ_EJ_BACK]=ON
"""
NORMAL_EXIT = """L PR[2:WAIT_IMM] 2000mm/sec FINE
WAIT DI[5:EJ_BACK]=ON TIMEOUT,LBL[99]
DO[3:REQ_EJ_BACK]=OFF
!Closure only from the start
!position, with the part
WAIT DO[10:MOULD_AREA_FREE]=ON TIMEOUT,LBL[99]
IF RI[1:PART_OK]=OFF,JMP LBL[99]
DO[1:REQ_CLOSE]=ON
"""
EARLY_EXIT = """!WEAKENED: closure requested as
!soon as the ejector is back,
!the retreat runs in parallel
WAIT DI[5:EJ_BACK]=ON TIMEOUT,LBL[99]
DO[3:REQ_EJ_BACK]=OFF
DO[1:REQ_CLOSE]=ON
L PR[2:WAIT_IMM] 2000mm/sec FINE
"""
REST = """!Downstream, mould is closing
J PR[5:CUT] 100% FINE
DO[4:AT_CUT]=ON
WAIT DI[6:CUT_DONE]=ON TIMEOUT,LBL[99]
DO[4:AT_CUT]=OFF
WAIT DI[7:PLACE_FREE]=ON TIMEOUT,LBL[99]
J PR[6:PLACE_APPR] 100% FINE
L PR[7:PLACE] 1000mm/sec FINE
RO[1:VACUUM]=OFF
WAIT RI[1:PART_OK]=OFF TIMEOUT,LBL[99]
L PR[6:PLACE_APPR] 1000mm/sec FINE
DO[5:PLACED]=ON
WAIT DI[9:PLACE_ACK]=ON TIMEOUT,LBL[99]
DO[5:PLACED]=OFF
!Finish the closure handshake
WAIT DI[3:MOULD_CLOSED]=ON TIMEOUT,LBL[99]
DO[1:REQ_CLOSE]=OFF
WAIT DI[3:MOULD_CLOSED]=OFF TIMEOUT,LBL[99]
J PR[2:WAIT_IMM] 100% FINE
JMP LBL[1]
LBL[99:FAULT]
DO[1:REQ_CLOSE]=OFF
DO[2:REQ_EJ_FWD]=OFF
DO[6:FAULT]=ON
UALM[1]
"""


def write(path: str, name: str, comment: str, body: str) -> None:
    lines = body.strip().split("\n")
    out = HEAD.format(name=name, comment=comment, n=len(lines))
    out += "".join(f"{i + 1:4d}:  {ln} ;\n" for i, ln in enumerate(lines)) + "/POS\n/END\n"
    (ROOT / path).write_text(out, encoding="utf-8", newline="\n")
    print(path, len(lines), "lines")


if __name__ == "__main__":
    write("robot/UNLOAD_IMM.ls", "UNLOAD_IMM", "Unload IMM", INIT + NORMAL_EXIT + REST)
    write("robot/variants/UNLOAD_EARLY.ls", "UNLOAD_EARLY", "WEAK early enable", INIT + EARLY_EXIT + REST)

/PROG  UNLOAD_IMM
/ATTR
OWNER		= MNEDITOR;
COMMENT		= "Unload IMM";
PROG_SIZE	= 1618;
CREATE		= DATE 26-11-25  TIME 13:52:28;
MODIFIED	= DATE 26-11-25  TIME 13:52:28;
FILE_NAME	= ;
VERSION		= 0;
LINE_COUNT	= 72;
MEMORY_SIZE	= 2122;
PROTECT		= READ_WRITE;
TCD:  STACK_SIZE	= 0,
      TASK_PRIORITY	= 50,
      TIME_SLICE	= 0,
      BUSY_LAMP_OFF	= 0,
      ABORT_REQUEST	= 0,
      PAUSE_REQUEST	= 0;
DEFAULT_GROUP	= 1,*,*,*,*;
CONTROL_CODE	= 00000000 00000000;
LOCAL_REGISTERS	= 0,0,0;
/MN
   1:  !Unload the mould of an injection ;
   2:  !moulding machine. Handshake with ;
   3:  !PLC FB CELL_PLC (EUROMAP 67) ;
   4:  !Signals: robot/io_map.yaml ;
   5:  UFRAME_NUM=1 ;
   6:  UTOOL_NUM=1 ;
   7:  $WAITTMOUT=500 ;
   8:  DO[1:REQ_CLOSE]=OFF ;
   9:  DO[2:REQ_EJ_FWD]=OFF ;
  10:  DO[3:REQ_EJ_BACK]=OFF ;
  11:  DO[4:AT_CUT]=OFF ;
  12:  DO[5:PLACED]=OFF ;
  13:  DO[6:FAULT]=OFF ;
  14:  DO[7:AT_WAIT]=OFF ;
  15:J PR[2:WAIT_IMM] 100% FINE    ;
  16:  LBL[1:CYCLE] ;
  17:  IF DI[8:CYCLE_ENABLE]=ON,JMP LBL[2] ;
  18:  !Access requested: park here ;
  19:  DO[7:AT_WAIT]=ON ;
  20:  WAIT DI[8:CYCLE_ENABLE]=ON    ;
  21:  DO[7:AT_WAIT]=OFF ;
  22:  JMP LBL[1] ;
  23:  LBL[2:RUN] ;
  24:  WAIT DI[1:IMM_AUTO]=ON    ;
  25:  WAIT (DI[2:MOULD_OPEN]=ON OR DI[8:CYCLE_ENABLE]=OFF)    ;
  26:  IF DI[8:CYCLE_ENABLE]=OFF,JMP LBL[1] ;
  27:  !Last check before the move ;
  28:  IF DI[2:MOULD_OPEN]=OFF,JMP LBL[1] ;
  29:  !Enter: DO[10] goes OFF when the ;
  30:  !robot leaves the start position ;
  31:L PR[3:MOULD_IN] 2000mm/sec FINE    ;
  32:L PR[4:GRIP] 500mm/sec FINE    ;
  33:  RO[1:VACUUM]=ON ;
  34:  DO[2:REQ_EJ_FWD]=ON ;
  35:  WAIT DI[4:EJ_FWD]=ON TIMEOUT,LBL[99] ;
  36:  WAIT RI[1:PART_OK]=ON TIMEOUT,LBL[99] ;
  37:  DO[2:REQ_EJ_FWD]=OFF ;
  38:L PR[3:MOULD_IN] 500mm/sec FINE    ;
  39:  DO[3:REQ_EJ_BACK]=ON ;
  40:L PR[2:WAIT_IMM] 2000mm/sec FINE    ;
  41:  WAIT DI[5:EJ_BACK]=ON TIMEOUT,LBL[99] ;
  42:  DO[3:REQ_EJ_BACK]=OFF ;
  43:  !Closure only from the start ;
  44:  !position, with the part ;
  45:  WAIT DO[10:MOULD_AREA_FREE]=ON TIMEOUT,LBL[99] ;
  46:  IF RI[1:PART_OK]=OFF,JMP LBL[99] ;
  47:  DO[1:REQ_CLOSE]=ON ;
  48:  !Downstream, mould is closing ;
  49:J PR[5:CUT] 100% FINE    ;
  50:  DO[4:AT_CUT]=ON ;
  51:  WAIT DI[6:CUT_DONE]=ON TIMEOUT,LBL[99] ;
  52:  DO[4:AT_CUT]=OFF ;
  53:  WAIT DI[7:PLACE_FREE]=ON TIMEOUT,LBL[99] ;
  54:J PR[6:PLACE_APPR] 100% FINE    ;
  55:L PR[7:PLACE] 1000mm/sec FINE    ;
  56:  RO[1:VACUUM]=OFF ;
  57:  WAIT RI[1:PART_OK]=OFF TIMEOUT,LBL[99] ;
  58:L PR[6:PLACE_APPR] 1000mm/sec FINE    ;
  59:  DO[5:PLACED]=ON ;
  60:  WAIT DI[9:PLACE_ACK]=ON TIMEOUT,LBL[99] ;
  61:  DO[5:PLACED]=OFF ;
  62:  !Finish the closure handshake ;
  63:  WAIT DI[3:MOULD_CLOSED]=ON TIMEOUT,LBL[99] ;
  64:  DO[1:REQ_CLOSE]=OFF ;
  65:  WAIT DI[3:MOULD_CLOSED]=OFF TIMEOUT,LBL[99] ;
  66:J PR[2:WAIT_IMM] 100% FINE    ;
  67:  JMP LBL[1] ;
  68:  LBL[99:FAULT] ;
  69:  DO[1:REQ_CLOSE]=OFF ;
  70:  DO[2:REQ_EJ_FWD]=OFF ;
  71:  DO[6:FAULT]=ON ;
  72:  UALM[1] ;
/POS
/END

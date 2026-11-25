(*
  CELL_PLC: cell controller between an injection moulding machine and a 6-axis unloading robot.
  IEC 61131-3 Structured Text, compiled with matiec (iec2c). The compiled block is the PLC in the timed
  in-the-loop run (cell/loop.py) and in the state-space exploration (cell/checker.py). The robot program
  robot/UNLOAD_IMM.ls is the other side of the handshake (signal list in robot/io_map.yaml).

  Machine side: logical signals of EUROMAP 67, version 1.11 (contact numbers in the names).
    in   ZA6 Mould closed, ZA7 Mould open position, ZB2 Enable operation with handling device / robot,
         ZB3 Ejector back position, ZB4 Ejector forward position
    out  A6 Enable mould closure, B3 Enable ejector back, B4 Enable ejector forward,
         B2 Handling device / robot operation mode (LOW = operation with injection moulding machine)
  Not in this block: the emergency stop channels (ZA1/ZC1, ZA2/ZC2, A1/C1, A2/C2), Safety devices of machine
  (ZA3/ZC3, ZA4/ZC4) and Mould area free (A3/C3). They are contact signals between the machine and the robot
  safety circuits. This block only reads monitoring copies (iSafetyOk, rbAreaFree).

  THIS IS STANDARD PLC CODE, NOT A SAFETY FUNCTION. A6 is a process interlock. Keeping the mould from
  closing on the robot is the job of the Mould area free contact and of the machine.

  Closure handshake (four phases, no timing assumption):
    robot REQ_CLOSE on -> A6 on -> ZA6 Mould closed -> MOULD_CLOSED on -> robot REQ_CLOSE off -> MOULD_CLOSED off
  The next "mould open" is passed to the robot only after the last phase.
*)

FUNCTION_BLOCK CELL_PLC
VAR_INPUT
  iZA6_MouldClosed : BOOL;
  iZA7_MouldOpenPosition : BOOL;
  iZB2_EnableOperation : BOOL;
  iZB3_EjectorBack : BOOL;
  iZB4_EjectorForward : BOOL;
  iSafetyOk : BOOL;            (* monitoring copy: e-stops, safety devices of machine, cell gate *)
  rbReqClose : BOOL;           (* robot DO[1]: retracted, part in the gripper, close the mould *)
  rbReqEjFwd : BOOL;           (* robot DO[2] *)
  rbReqEjBack : BOOL;          (* robot DO[3] *)
  rbAtCut : BOOL;              (* robot DO[4]: part presented to the sprue cutter *)
  rbPlaced : BOOL;             (* robot DO[5]: part placed on the nest *)
  rbFault : BOOL;              (* robot DO[6] *)
  rbAtWait : BOOL;             (* robot DO[7]: parked at the start position, cycle stopped *)
  rbAreaFree : BOOL;           (* robot DO[10]: monitoring copy of Mould area free (zone signal) *)
  iCutClosed : BOOL;
  iCutOpen : BOOL;
  iNestFree : BOOL;
  iGateClosedLocked : BOOL;
  btnGateReq : BOOL;
  btnReset : BOOL;
  cfgCloseTmoScans : INT;      (* watchdog of the closing stroke in scans; 0 = off *)
END_VAR
VAR_OUTPUT
  qA6_EnableMouldClosure : BOOL;
  qB3_EnableEjectorBack : BOOL;
  qB4_EnableEjectorForward : BOOL;
  qB2_NoOperationWithImm : BOOL;
  qRbImmAuto : BOOL;           (* robot DI[1] *)
  qRbMouldOpen : BOOL;         (* robot DI[2] *)
  qRbMouldClosed : BOOL;       (* robot DI[3]: closure acknowledged *)
  qRbEjFwd : BOOL;             (* robot DI[4] *)
  qRbEjBack : BOOL;            (* robot DI[5] *)
  qRbCutDone : BOOL;           (* robot DI[6] *)
  qRbPlaceFree : BOOL;         (* robot DI[7] *)
  qRbCycleEnable : BOOL;       (* robot DI[8] *)
  qRbPlaceAck : BOOL;          (* robot DI[9]: PLACED seen, index pending *)
  qCutClose : BOOL;
  qConvIndex : BOOL;
  qGateUnlock : BOOL;
  state : INT;
  faultCode : INT;
END_VAR
VAR CONSTANT
  S_INIT : INT := 0;
  S_MOULDING : INT := 1;
  S_OPEN : INT := 2;
  S_UNLOAD : INT := 3;
  S_CLOSE : INT := 4;
  S_ACCESS : INT := 5;
  S_FAULT : INT := 9;
  F_SAFETY : INT := 1;
  F_ROBOT : INT := 2;
  F_SENSOR : INT := 3;
  F_SEQUENCE : INT := 4;
  F_AREA : INT := 5;
  F_CLOSE_TMO : INT := 6;
END_VAR
VAR
  closedAck : BOOL;
  cutDone : BOOL;
  idx : BOOL;
  gateReq : BOOL;
  accessEnd : BOOL;
  tmo : INT;
  mouldOpen : BOOL;
  running : BOOL;
  newFault : INT;
  seq : INT;                   (* sequence state; copied to the output 'state' at the end *)
END_VAR

  mouldOpen := iZA7_MouldOpenPosition AND NOT iZA6_MouldClosed;
  IF btnGateReq THEN
    gateReq := TRUE;
  END_IF;

  (* supervision, valid in every running state *)
  newFault := 0;
  IF seq <> S_FAULT AND seq <> S_INIT THEN
    IF NOT iSafetyOk AND seq <> S_ACCESS THEN
      newFault := F_SAFETY;
    ELSIF rbFault THEN
      newFault := F_ROBOT;
    ELSIF iZA6_MouldClosed AND iZA7_MouldOpenPosition THEN
      newFault := F_SENSOR;
    END_IF;
  END_IF;

  IF newFault <> 0 THEN
    faultCode := newFault;
    seq := S_FAULT;
  ELSE
    CASE seq OF
      0: (* S_INIT: start only from a clean handshake *)
        IF iSafetyOk AND iGateClosedLocked AND rbAreaFree AND NOT rbReqClose AND NOT rbFault THEN
          closedAck := FALSE;
          accessEnd := FALSE;
          gateReq := FALSE;
          faultCode := 0;
          seq := S_MOULDING;
        END_IF;

      1: (* S_MOULDING: mould closed or moving, robot outside *)
        IF closedAck AND NOT rbReqClose THEN
          closedAck := FALSE;
        ELSIF gateReq AND rbAtWait AND rbAreaFree AND NOT closedAck THEN
          accessEnd := FALSE;
          seq := S_ACCESS;
        ELSIF mouldOpen AND NOT rbReqClose AND NOT closedAck THEN
          seq := S_OPEN;
        END_IF;

      2: (* S_OPEN: mould at the open position, robot at its start position *)
        IF NOT rbAreaFree THEN
          seq := S_UNLOAD;
        ELSIF NOT mouldOpen THEN
          faultCode := F_SEQUENCE;
          seq := S_FAULT;
        ELSIF gateReq AND rbAtWait THEN
          accessEnd := FALSE;
          seq := S_ACCESS;
        END_IF;

      3: (* S_UNLOAD: robot in the mould area *)
        IF NOT mouldOpen THEN
          faultCode := F_SEQUENCE;
          seq := S_FAULT;
        ELSIF rbAreaFree AND rbReqClose AND iZB3_EjectorBack THEN
          tmo := 0;
          seq := S_CLOSE;
        END_IF;

      4: (* S_CLOSE: A6 is on until the machine reports Mould closed *)
        IF iZA6_MouldClosed THEN
          closedAck := TRUE;
          seq := S_MOULDING;
        ELSIF NOT rbAreaFree THEN
          faultCode := F_AREA;
          seq := S_FAULT;
        ELSIF NOT rbReqClose THEN
          faultCode := F_SEQUENCE;
          seq := S_FAULT;
        ELSIF cfgCloseTmoScans > 0 THEN
          tmo := tmo + 1;
          IF tmo > cfgCloseTmoScans THEN
            faultCode := F_CLOSE_TMO;
            seq := S_FAULT;
          END_IF;
        END_IF;

      5: (* S_ACCESS: gate released on request, robot parked at the start position *)
        IF btnReset THEN
          accessEnd := TRUE;
        END_IF;
        IF accessEnd AND iGateClosedLocked AND iSafetyOk THEN
          gateReq := FALSE;
          accessEnd := FALSE;
          seq := S_MOULDING;
        END_IF;

      9: (* S_FAULT: every enable is off; restart only from a clean handshake *)
        IF btnReset AND iSafetyOk AND iGateClosedLocked AND rbAreaFree AND NOT rbReqClose AND NOT rbFault THEN
          seq := S_INIT;
        END_IF;
    END_CASE;
  END_IF;

  running := seq = S_MOULDING OR seq = S_OPEN OR seq = S_UNLOAD OR seq = S_CLOSE;

  (* EUROMAP 67 outputs *)
  qA6_EnableMouldClosure := seq = S_CLOSE AND rbReqClose AND rbAreaFree AND iZB3_EjectorBack AND iSafetyOk;
  qB4_EnableEjectorForward := seq = S_UNLOAD AND rbReqEjFwd AND mouldOpen AND NOT rbAreaFree;
  qB3_EnableEjectorBack := seq = S_UNLOAD AND rbReqEjBack;
  qB2_NoOperationWithImm := FALSE;

  (* to the robot *)
  qRbImmAuto := running AND iZB2_EnableOperation;
  qRbMouldOpen := (seq = S_OPEN OR seq = S_UNLOAD) AND mouldOpen;
  qRbMouldClosed := closedAck;
  qRbEjFwd := iZB4_EjectorForward AND NOT iZB3_EjectorBack;
  qRbEjBack := iZB3_EjectorBack AND NOT iZB4_EjectorForward;
  qRbCycleEnable := running AND NOT gateReq;

  (* sprue cutter: close once per presentation, report when it is open again *)
  IF NOT rbAtCut THEN
    cutDone := FALSE;
  ELSIF iCutClosed THEN
    cutDone := TRUE;
  END_IF;
  qCutClose := running AND iSafetyOk AND rbAtCut AND NOT cutDone;
  qRbCutDone := rbAtCut AND cutDone AND iCutOpen;

  (* conveyor: latch PLACED, acknowledge it, index one nest after the robot has taken its signal back *)
  IF rbPlaced THEN
    idx := TRUE;
  END_IF;
  IF idx AND NOT rbPlaced AND iNestFree THEN
    idx := FALSE;
  END_IF;
  qConvIndex := running AND iSafetyOk AND idx AND NOT rbPlaced;
  qRbPlaceFree := iNestFree AND NOT idx;
  qRbPlaceAck := idx;

  (* gate *)
  qGateUnlock := seq = S_ACCESS AND NOT accessEnd;

  (* outputs are written once and never read back *)
  state := seq;
END_FUNCTION_BLOCK

# Changelog

## 1.0.0 (2026-11-25)
- Cooling time of the handle section (plate relation, finite-difference check), cycle breakdown, annual output.
- Removal time of a 6-axis robot from joint speed and acceleration limits; trade study against a 3-axis linear robot.
- EUROMAP 67 (version 1.11) signal interface; cell PLC in IEC 61131-3 ST compiled with matiec, TIA Portal SCL
  export; FANUC TP program in LS format and its interpreter.
- Machine, robot program and PLC as communicating state machines; complete state-space exploration for the
  fault-free cell and 44 single faults; weakened variants with counter-example traces; liveness check.
- Timed in-the-loop run with the same transition function; timing diagram of the handshake.
- Dossier, figures, tests, CI with matiec.

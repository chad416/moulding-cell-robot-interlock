# Changelog

## 1.2.0 (2026-11-25)
- `robot/UNLOAD_IMM.ls` translated and run on a FANUC virtual controller (R-30iB Plus virtual controller, software
  V9.40, arm M-710iD/70) against the machine model and the PLC block: run script, bridge, evaluation, records,
  timing figure. One TP comment shortened to 32 characters so that the controller's listing equals the source.

## 1.1.1 (2026-11-25)
- Test bench run in S7-PLCSIM Advanced V7.0 Upd1 under TIA Portal V20 Update 7 (project upgraded from V18; two CPU
  protection settings set by hand so that the hardware compiles): 7 of 7 cases passed. Record, observed values and
  screenshots in `docs/PLCSIM_RUN.md`.
- `scripts/make_testbench.py` keeps the hand-written result section of `docs/PLCSIM_RUN.md` when it writes the page.

## 1.1.0 (2026-11-25)
- Self-checking PLC test bench for PLCSIM (`plc/test`): seven cases, tables generated from the cell model,
  result data block `DB_TestResult`. Runs against the matiec build in CI; a broken expectation is shown to fail.
- `scripts/tia_build.ps1`: any SCL sources by path, own OB1, project archive.

## 1.0.1 (2026-11-25)
- PLC block: the sequence state is an internal variable and is copied to the output `state` at the end, so no
  output is read back. The SCL export compiles in TIA Portal V18 with 0 errors and 0 warnings
  (`results/tia_compile.txt`). No result number changed.
- `scripts/tia_build.ps1` accepts a relative source folder.

## 1.0.0 (2026-11-25)
- Cooling time of the handle section (plate relation, finite-difference check), cycle breakdown, annual output.
- Removal time of a 6-axis robot from joint speed and acceleration limits; trade study against a 3-axis linear robot.
- EUROMAP 67 (version 1.11) signal interface; cell PLC in IEC 61131-3 ST compiled with matiec, TIA Portal SCL
  export; FANUC TP program in LS format and its interpreter.
- Machine, robot program and PLC as communicating state machines; complete state-space exploration for the
  fault-free cell and 44 single faults; weakened variants with counter-example traces; liveness check.
- Timed in-the-loop run with the same transition function; timing diagram of the handshake.
- Dossier, figures, tests, CI with matiec.

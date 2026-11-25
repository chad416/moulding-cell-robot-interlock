<#
  Run robot\UNLOAD_IMM.ls on a FANUC virtual controller (ROBOGUIDE) against the repository's machine model and
  compiled PLC block.

  What runs where
    virtual controller   translates the LS file itself (FTP upload) and executes the TP program
    this script          reads the robot outputs and the tool position, writes the simulated robot inputs, and plays
                         the zone function: DO[10] is ON while the tool is not beyond the start position towards
                         the mould
    WSL                  scripts/vc_plc_bridge.py: machine and cell-device model, PLC block compiled with matiec

  Needs: a virtual controller that is already running on this PC with its web server on port 9000, FTP and the
  PC-interface library (FRRobot); no background logic or other program that writes DO[1..7], DO[10]; WSL with the
  repository's Python packages and build/plc/libplc.so. It talks to 127.0.0.1 only.

  What it changes on the controller: comments and simulation of DI[1..9], RI[1]; comments of DO[1..7], DO[10],
  RO[1]; user frame 1 (set to zero); PR[1..7] (positions around the current tool position, see $Layout);
  $UALRM_MSG[1]; active frame and tool numbers; the program UNLOAD_IMM.

  Usage:  powershell -NoProfile -ExecutionPolicy Bypass -File scripts\vc_run.ps1 [-Cycles 3] [-ProcessTime 6]
  Result: results\virtual_controller_run.jsonl (raw log), results\virtual_controller_listing.ls (the program as
  the controller lists it). Then: python -m scripts.vc_report
#>
param([int]$Cycles = 3, [double]$ProcessTime = 6.0, [double]$HoldOpen = 12.0, [double]$ZoneMargin = 30.0,
      [string]$Distro = "Ubuntu-24.04", [string]$Program = "UNLOAD_IMM")
$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $PSScriptRoot
$log = Join-Path $root "results\virtual_controller_run.jsonl"
$base = "http://127.0.0.1:9000"
$sw = [System.Diagnostics.Stopwatch]::StartNew()
$lines = New-Object System.Collections.ArrayList
$script:tc = 0
function Rec($obj) { $obj["t"] = [math]::Round($sw.Elapsed.TotalSeconds, 3); $obj["tc"] = $script:tc; [void]$lines.Add(($obj | ConvertTo-Json -Compress -Depth 5)) }
function Note($msg) { Rec @{ note = $msg }; Write-Host ("{0,7:N1} s  {1}" -f $sw.Elapsed.TotalSeconds, $msg) }
function WebGet($url) { (& curl.exe -s -m 15 $url 2>$null) -join "`n" }
function KCL($cmd) {
  $t = WebGet ($base + "/KCL/" + [uri]::EscapeDataString($cmd))
  $m = [regex]::Match($t, '(?s)<XMP>(.*?)</XMP>'); if ($m.Success) { ($m.Groups[1].Value -replace '\s+', ' ').Trim() } else { '' }
}
function VarValue($name) { ((KCL "SHOW VAR $name") -replace '^.*=\s*', '').Trim("' ") }

# offsets from the tool position at the start (mm, world): the layout of the study, shortened to fit any arm
$Layout = [ordered]@{ HOME = @(-100, 0, 100); WAIT_IMM = @(0, 0, 0); MOULD_IN = @(500, 0, 0); GRIP = @(500, 90, 0)
                      CUT = @(-300, 350, -100); PLACE_APPR = @(-300, 900, 0); PLACE = @(-300, 900, -200) }
$PrNum = @{ HOME = 1; WAIT_IMM = 2; MOULD_IN = 3; GRIP = 4; CUT = 5; PLACE_APPR = 6; PLACE = 7 }
$DiName = @{ 1 = 'IMM_AUTO'; 2 = 'MOULD_OPEN'; 3 = 'MOULD_CLOSED'; 4 = 'EJ_FWD'; 5 = 'EJ_BACK'; 6 = 'CUT_DONE'; 7 = 'PLACE_FREE'; 8 = 'CYCLE_ENABLE'; 9 = 'PLACE_ACK' }
$DoName = @{ 1 = 'REQ_CLOSE'; 2 = 'REQ_EJ_FWD'; 3 = 'REQ_EJ_BACK'; 4 = 'AT_CUT'; 5 = 'PLACED'; 6 = 'FAULT'; 7 = 'AT_WAIT'; 10 = 'MOULD_AREA_FREE' }

$rob = New-Object -ComObject FRRobot.FRCRobot
$rob.ConnectEx('127.0.0.1', $false, 10, 2)
if (-not $rob.IsConnected) { throw "PC interface: not connected to 127.0.0.1" }
$version = VarValue '$VERSION'; $model = VarValue '$SCR_GRP[1].$ROBOT_MODEL'
Note "controller software $version, arm $model"
$bg = KCL 'SHOW VAR $MIX_BG[1].$STATUS'
if ($bg -match '=\s*[12]\s*$') { throw "background logic is running in slot 1; stop it first (it may write the outputs of this test)" }

# ---- set-up on the controller -----------------------------------------------------------------------------
foreach ($t in $rob.Tasks) { if (-not $t.SystemTask -and [int]$t.Status -ne 2) { try { $t.Abort() } catch {} } }
foreach ($c in 'SET VAR $MNUFRAMENUM[1] = 1', 'SET VAR $MNUTOOLNUM[1] = 1', 'SET VAR $MCR.$GENOVERRIDE = 100',
               'SET VAR $RMT_MASTER = 2', 'RESET') { [void](KCL $c) }
# the text of user alarm 1 goes through the PC interface (a string with quotes in a web KCL request stopped the
# controller's web server in one run)
try { $ua = $rob.SysVariables.Item('$UALRM_MSG[1]'); $ua.Value = 'UNLOAD_IMM handshake fault'; $uaSet = $true } catch { $uaSet = $false }
Note "text of user alarm 1 set: $uaSet"
$uf = $rob.UserFrames.Item(1); $f = $uf.Group(1).Formats(2)
$f.X = 0; $f.Y = 0; $f.Z = 0; $f.W = 0; $f.P = 0; $f.R = 0; $uf.Update()
$di = $rob.IOTypes.Item(1).Signals; $do = $rob.IOTypes.Item(2).Signals
$ri = $rob.IOTypes.Item(8).Signals.Item(1); $ro = $rob.IOTypes.Item(9).Signals.Item(1)
foreach ($n in $DiName.Keys) { $s = $di.Item($n); $s.Comment = $DiName[$n]; $s.Simulate = $true; $s.Value = $false }
foreach ($n in $DoName.Keys) { $s = $do.Item($n); $s.Comment = $DoName[$n]; $s.Simulate = $false; $s.Value = $false }
$ri.Comment = 'PART_OK'; $ri.Simulate = $true; $ri.Value = $false; $ro.Comment = 'VACUUM'; $ro.Value = $false
$cw = $rob.CurPosition.Group(1, 2); $cw.Refresh(); $w = $cw.Formats(2)
$wait = $rob.RegPositions.Item(2)
if ($wait.IsInitialized -and $wait.Group(1).Comment -eq 'WAIT_IMM') {
  $q = $wait.Group(1).Formats(2); $p0 = @([double]$q.X, [double]$q.Y, [double]$q.Z)
  Note "position registers kept from an earlier run"
} else {
  $p0 = @([double]$w.X, [double]$w.Y, [double]$w.Z)
  foreach ($name in $Layout.Keys) {
    $p = $rob.RegPositions.Item($PrNum[$name]); $p.Record(); $g = $p.Group(1); $x = $g.Formats(2)
    $x.X = $p0[0] + $Layout[$name][0]; $x.Y = $p0[1] + $Layout[$name][1]; $x.Z = $p0[2] + $Layout[$name][2]
    $g.Comment = $name; $p.Update()
  }
  Note "position registers 1 to 7 written around the tool position at the start"
}
$pts = @{}; foreach ($name in $Layout.Keys) { $pts[$name] = @(($p0[0] + $Layout[$name][0]), ($p0[1] + $Layout[$name][1]), ($p0[2] + $Layout[$name][2])) }

# ---- the controller translates the LS file ------------------------------------------------------------------
$ls = Join-Path $root ("robot\" + $Program + ".ls")
if (-not (Test-Path $ls)) { $ls = Join-Path $root ("robot\variants\" + $Program + ".ls") }
$tmp = Join-Path $env:TEMP ($Program + ".LS"); Copy-Item $ls $tmp -Force
$ErrorActionPreference = "Continue"
foreach ($try in 1..3) {
  $put = (& curl.exe -s -S -m 60 -T $tmp "ftp://127.0.0.1/" 2>&1) -join ' '; $putExit = $LASTEXITCODE
  if ($putExit -eq 0) { break }
  # "program is in use": an older copy is still open or selected; release it and try again
  try { $rob.Programs.Item($Program).Close() } catch {}
  try { $rob.Programs.Selected = 'PNLINST' } catch {}
  Start-Sleep 1
}
$ErrorActionPreference = "Stop"
Rec @{ ftp_put = (Split-Path $ls -Leaf); exit = $putExit; message = $put }
Write-Host "FTP put of $(Split-Path $ls -Leaf): exit $putExit $put"
if ($putExit -ne 0) { [IO.File]::WriteAllLines($log, $lines); throw "the controller did not accept the LS file" }
$listing = (& curl.exe -s -m 30 ("ftp://127.0.0.1/md:/" + $Program + ".LS")) -join "`n"
[IO.File]::WriteAllText((Join-Path $root "results\virtual_controller_listing.ls"), $listing + "`n")

# ---- machine model and PLC block (WSL) ----------------------------------------------------------------------
$wslRoot = "/mnt/" + $root.Substring(0, 1).ToLower() + ($root.Substring(2) -replace '\\', '/')
$psi = New-Object System.Diagnostics.ProcessStartInfo
$psi.FileName = "wsl.exe"; $psi.Arguments = "-d $Distro -- bash -lc `"cd '$wslRoot' && python3 -u -m scripts.vc_plc_bridge $ProcessTime`""
$psi.RedirectStandardInput = $true; $psi.RedirectStandardOutput = $true; $psi.UseShellExecute = $false; $psi.CreateNoWindow = $true
$bridge = [System.Diagnostics.Process]::Start($psi)

function Dist($a, $b) { [math]::Sqrt(($a[0] - $b[0]) * ($a[0] - $b[0]) + ($a[1] - $b[1]) * ($a[1] - $b[1]) + ($a[2] - $b[2]) * ($a[2] - $b[2])) }
function Task() { foreach ($t in $rob.Tasks) { if ($t.Name -eq $Program) { return $t } }; $null }

$doSig = @{}; foreach ($n in 1..7) { $doSig[$n] = $do.Item($n) }
$clock = $rob.SysVariables.Item('$FAST_CLOCK'); $pos = 'WAIT_IMM'
$rob.Programs.Item($Program).Run()
Note "program $Program started through the PC interface"
# all robot inputs are still OFF: the program moves to the start position and parks there
$deadline = (Get-Date).AddSeconds(40)
while ((Get-Date) -lt $deadline) {
  $cw.Refresh(); $w = $cw.Formats(2)
  if ((Dist @([double]$w.X, [double]$w.Y, [double]$w.Z) $pts['WAIT_IMM']) -lt 1.0) { break }
  Start-Sleep -Milliseconds 100
}
$bridge.StandardInput.WriteLine('{"t":0,"do":[],"ro":[],"pos":"WAIT_IMM","block":[]}'); $bridge.StandardInput.Flush()
if (-not $bridge.StandardOutput.ReadLine()) { throw "the PLC bridge did not start" }
$sw.Restart(); Note "robot at the start position; machine model and PLC block running"
$block = @(); $prev = @{}; $diNow = @{}; $riNow = $false; $closures = 0; $lastA6 = 0; $phase = 'normal'; $phaseT = 0.0
$stopAt = $null
while ($true) {
  $t = $sw.Elapsed.TotalSeconds
  $cw.Refresh(); $w = $cw.Formats(2); $xyz = @([double]$w.X, [double]$w.Y, [double]$w.Z)
  try { $clock.Refresh(); $script:tc = [int]$clock.Value } catch {}
  # zone function: the mould area is a box that starts $ZoneMargin mm beyond the start position
  $inside = ($xyz[0] -gt ($p0[0] + $ZoneMargin)) -and ([math]::Abs($xyz[1] - $p0[1]) -lt 200) -and ([math]::Abs($xyz[2] - $p0[2]) -lt 150)
  $pos = if ($inside) { 'MOULD_IN' } else { 'WAIT_IMM' }
  foreach ($name in 'GRIP', 'PLACE') { if ((Dist $xyz $pts[$name]) -lt 2.0) { $pos = $name } }
  $free = $do.Item(10); if ([bool]$free.Value -ne (-not $inside)) { $free.Value = (-not $inside) }
  $on = @(); foreach ($n in 1..7) { $sig = $doSig[$n]; $sig.Refresh(); if ($sig.Value) { $on += $n } }
  $ro.Refresh(); $roOn = @(); if ($ro.Value) { $roOn += 1 }
  $bl = if ($block.Count) { '"' + ($block -join '","') + '"' } else { '' }
  $bridge.StandardInput.WriteLine(('{{"t":{0},"do":[{1}],"ro":[{2}],"pos":"{3}","block":[{4}]}}' -f $t.ToString([Globalization.CultureInfo]::InvariantCulture), ($on -join ','), ($roOn -join ','), $pos, $bl))
  $bridge.StandardInput.Flush()
  $ans = $bridge.StandardOutput.ReadLine(); if (-not $ans) { Note "bridge ended"; break }
  $r = $ans | ConvertFrom-Json
  foreach ($n in 1..9) { $v = [bool]$r.di_num."$n"; if (-not $diNow.ContainsKey($n) -or $diNow[$n] -ne $v) { $di.Item($n).Value = $v; $diNow[$n] = $v } }
  if ([bool]$r.ri1 -ne $riNow) { $riNow = [bool]$r.ri1; $ri.Value = $riNow }
  $tk = Task
  $cur = [ordered]@{}
  foreach ($p in $r.sig.PSObject.Properties) { $cur[$p.Name] = [int]$p.Value }
  $cur['DO10_MOULD_AREA_FREE'] = [int](-not $inside); $cur['RO1_VACUUM'] = $roOn.Count; $cur['RI1_PART_OK'] = [int]$riNow
  $cur['mould'] = $r.mould; $cur['ejector'] = $r.ej; $cur['plc'] = $r.plc; $cur['plc_fault'] = $r.plc_fault; $cur['pos'] = $pos
  $cur['line'] = if ($tk) { [int]$tk.CurLine } else { 0 }; $cur['task'] = if ($tk) { [int]$tk.Status } else { 2 }
  $cur['p1'] = [int]$r.p1; $cur['p2'] = [int]$r.p2
  $chg = @{}; foreach ($k in $cur.Keys) { if (-not $prev.ContainsKey($k) -or $prev[$k] -ne $cur[$k]) { $chg[$k] = $cur[$k]; $prev[$k] = $cur[$k] } }
  if ($chg.Count) { Rec @{ chg = $chg; dxyz = @([math]::Round($xyz[0] - $p0[0], 1), [math]::Round($xyz[1] - $p0[1], 1), [math]::Round($xyz[2] - $p0[2], 1)) } }
  foreach ($e in $r.events) { Rec @{ machine = $e[1]; tm = $e[0] } }

  # ---- scenario control ----
  if ($cur['A6'] -eq 1 -and $lastA6 -eq 0) { $closures++; Note "closure $closures enabled (A6), phase $phase" }
  $lastA6 = $cur['A6']
  if ($phase -eq 'normal' -and $closures -ge $Cycles -and $cur['ZA6'] -eq 1) { $phase = 'hold_open'; $phaseT = $t; $block = @('imm:open_start'); Note "scenario: the mould is kept closed for $HoldOpen s (mould open never comes)" }
  elseif ($phase -eq 'hold_open' -and ($t - $phaseT) -ge ($HoldOpen + $ProcessTime)) { $phase = 'one_more'; $block = @(); Note "scenario: mould released, robot was at line $($cur['line']) the whole time: $($cur['pos'])" }
  elseif ($phase -eq 'one_more' -and $closures -ge ($Cycles + 1) -and $cur['ZA6'] -eq 1) { $phase = 'no_ejector'; $block = @('imm:ej_fwd_done'); Note "scenario: the ejector will not move forward (WAIT time-out path)" }
  elseif ($phase -eq 'no_ejector' -and $cur['FAULT'] -eq 1 -and -not $stopAt) { $stopAt = $t + 3.0; Note "robot reports FAULT (DO[6]) at line $($cur['line'])" }
  if ($stopAt -and $t -ge $stopAt) { break }
  if ($cur['task'] -eq 2 -and $t -gt 3 -and -not $stopAt) { Note "the program is not running (aborted); see the alarms"; break }
  if ($t -gt 400) { Note "time limit"; break }
}
$tk = Task
Note ("end: task " + $(if ($tk) { "line $($tk.CurLine) status $($tk.Status)" } else { "gone" }))
$al = $rob.Alarms; $t0 = (Get-Date).AddSeconds(-$sw.Elapsed.TotalSeconds - 5)
foreach ($i in 1..8) {
  try {
    $a = $al.Item($i)
    # only alarms that name this program (the controller clock is simulation time, so no filter by time)
    $msg = ("" + $a.ErrorMessage).Trim(); $cause = ("" + $a.CauseMessage).Trim()
    if (($msg + $cause) -match [regex]::Escape($Program)) { Rec @{ alarm = ("" + $a.ErrorMnemonic + " " + $msg); cause = $cause } }
  } catch { Rec @{ note = "alarm $i not readable" } }
}
if ($tk) { try { $tk.Abort() } catch {} }
foreach ($n in $DoName.Keys) { try { $do.Item($n).Value = $false } catch {} }
try { $bridge.StandardInput.WriteLine('{"quit":1}'); $bridge.StandardInput.Flush(); [void]$bridge.WaitForExit(5000) } catch {}
Rec @{ meta = @{ software = $version; arm = $model; cycles = $Cycles; process_time_s = $ProcessTime; hold_open_s = $HoldOpen; program = $Program; layout_offsets_mm = $Layout; zone_margin_mm = $ZoneMargin } }
[IO.File]::WriteAllLines($log, $lines)
Write-Host "log: $log ($($lines.Count) records)"

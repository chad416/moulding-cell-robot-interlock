<#
  Generate the function blocks from plc\*.scl in a new TIA Portal V18 project and compile.

  Needs: TIA Portal V18 with STEP 7 Professional (licence), the Windows user in the
  "Siemens TIA Openness" group, Windows PowerShell 5.1.

  Usage:  powershell -ExecutionPolicy Bypass -File scripts\tia_build.ps1 -Blocks FB_A,FB_B
  Blocks are generated in the order given (a called block before its caller).
  Result: the compile messages, printed and written to results\tia_compile.txt.
#>
param(
    [Parameter(Mandatory = $true)][string[]]$Blocks,
    [string]$PlcDir = "",
    [string]$OutDir = "C:\TIA_build",
    [string]$ResultFile = "",
    [string]$Version = "V18",
    [string]$Cpu = "OrderNumber:6ES7 513-1AL02-0AB0/V2.9"
)
$ErrorActionPreference = "Stop"
$Blocks = @($Blocks | ForEach-Object { $_ -split "," } | ForEach-Object { $_.Trim() } | Where-Object { $_ })
$root = Split-Path -Parent $PSScriptRoot
if ($PlcDir -eq "") { $PlcDir = Join-Path $root "plc" }
$PlcDir = (Resolve-Path $PlcDir).Path   # TIA Portal runs in its own process: it needs a full path
if ($ResultFile -eq "") { $ResultFile = Join-Path $root "results\tia_compile.txt" }
$api = "C:\Program Files\Siemens\Automation\Portal $Version\PublicAPI\$Version\Siemens.Engineering.dll"
if (-not (Test-Path $api)) { throw "Openness API not found: $api" }

$code = @"
using System;
using System.IO;
using System.Linq;
using System.Text;
using Siemens.Engineering;
using Siemens.Engineering.HW;
using Siemens.Engineering.HW.Features;
using Siemens.Engineering.SW;
using Siemens.Engineering.SW.ExternalSources;
using Siemens.Engineering.Compiler;

public static class SclBuild {
    static void Walk(CompilerResultMessage m, StringBuilder sb, int depth) {
        sb.AppendLine(new string(' ', 2 * depth) + m.State + "  " + m.Path + "  " + m.Description);
        foreach (var c in m.Messages) Walk(c, sb, depth + 1);
    }
    public static string Run(string outDir, string plcDir, string cpu, string[] blocks) {
        var sb = new StringBuilder();
        using (var tia = new TiaPortal(TiaPortalMode.WithoutUserInterface)) {
            var dir = new DirectoryInfo(outDir);
            var project = tia.Projects.Create(dir, "build_" + DateTime.Now.ToString("yyyyMMdd_HHmmss"));
            var dev = project.Devices.CreateWithItem(cpu, "PLC_1", "PLC_1");
            PlcSoftware plc = null;
            foreach (var item in dev.DeviceItems) {
                var sc = item.GetService<SoftwareContainer>();
                if (sc != null && sc.Software is PlcSoftware) { plc = (PlcSoftware)sc.Software; break; }
            }
            if (plc == null) throw new Exception("no PLC software in " + cpu);
            foreach (var name in blocks) {
                var src = plc.ExternalSourceGroup.ExternalSources.CreateFromFile(
                    name, Path.Combine(plcDir, name + ".scl"));
                try {
                    src.GenerateBlocksFromSource();
                    sb.AppendLine("generated " + name);
                } catch (Exception ex) {
                    sb.AppendLine("GENERATE FAILED " + name + ": " + (ex.InnerException ?? ex).Message);
                }
            }
            var result = plc.GetService<ICompilable>().Compile();
            sb.AppendLine("COMPILE " + result.State + "  errors=" + result.ErrorCount + "  warnings=" + result.WarningCount);
            foreach (var m in result.Messages) Walk(m, sb, 1);
            project.Save();
            project.Close();
        }
        return sb.ToString();
    }
}
"@

$resolver = @"
using System; using System.IO; using System.Reflection;
public static class OpnResolve {
    static string[] dirs;
    public static void Init(string[] d) {
        dirs = d;
        AppDomain.CurrentDomain.AssemblyResolve += (s, e) => {
            var n = new AssemblyName(e.Name).Name;
            if (n.EndsWith(".resources")) return null;
            foreach (var dir in dirs) { var p = Path.Combine(dir, n + ".dll"); if (File.Exists(p)) return Assembly.LoadFrom(p); }
            return null;
        };
    }
}
"@
Add-Type -TypeDefinition $resolver
$portal = "C:\Program Files\Siemens\Automation\Portal $Version"
[OpnResolve]::Init(@((Split-Path -Parent $api), "$portal\Bin\PublicAPI", "$portal\Bin"))
New-Item -ItemType Directory -Force $OutDir | Out-Null
[void][System.Reflection.Assembly]::LoadFrom($api)
Add-Type -TypeDefinition $code -ReferencedAssemblies $api
$text = "TIA Portal $Version, CPU $Cpu, " + (Get-Date -Format "yyyy-MM-dd HH:mm") + "`r`n" + [SclBuild]::Run($OutDir, $PlcDir, $Cpu, $Blocks)
$text
$text | Out-File -Encoding utf8 $ResultFile

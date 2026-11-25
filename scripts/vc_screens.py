"""Records and pictures from the virtual controller after scripts/vc_run.ps1 (Windows, controller on 127.0.0.1).

    python -m scripts.vc_screens

Reads over FTP what the controller itself reports: its listing of the program, its active-alarm file and its I/O
status file. The I/O status is cut down to the signals of this test. The texts are saved under results/ and
rendered to PNG with a headless browser (no window): docs/img/vc_program_listing.png, docs/img/vc_alarm_io.png.
These pictures are renderings of text returned by the controller, not captures of a pendant or simulator window.
"""

from __future__ import annotations

import html
import re
import subprocess
import tempfile
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
RES, IMG = ROOT / "results", ROOT / "docs" / "img"
EDGE = [r"C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe", r"C:\Program Files\Microsoft\Edge\Application\msedge.exe"]
KEEP = re.compile(r"^(DIN\[\s*[1-9]\]|DOUT\[\s*([1-7]|10)\]|RDI\[\s*1\]|RDO\[\s*1\])")
CSS = ("body{font-family:Consolas,monospace;font-size:13px;background:#fcfcfb;color:#0b0b0b;margin:18px}"
       "h1{font-family:Segoe UI,Arial,sans-serif;font-size:15px;margin:0 0 4px}"
       "p{font-family:Segoe UI,Arial,sans-serif;font-size:12px;color:#52514e;margin:0 0 10px}"
       "pre{margin:0 0 16px;line-height:1.32}")


def ftp(name: str) -> str:
    out = subprocess.run(["curl.exe", "-s", "-m", "30", f"ftp://127.0.0.1/md:/{name}"], capture_output=True, check=True)
    return out.stdout.decode("latin-1").replace("\r\n", "\n")


def shot(title: str, note: str, blocks: list[tuple[str, str]], png: Path, size: str) -> None:
    body = f"<h1>{html.escape(title)}</h1><p>{html.escape(note)}</p>"
    for head, text in blocks:
        body += f"<p><b>{html.escape(head)}</b></p><pre>{html.escape(text)}</pre>"
    if png.exists():
        png.unlink()
    with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as tmp:
        page = Path(tmp) / "page.html"
        page.write_text(f"<!doctype html><meta charset='utf-8'><style>{CSS}</style>{body}", encoding="utf-8")
        edge = next(e for e in EDGE if Path(e).exists())
        subprocess.run([edge, "--headless=new", "--disable-gpu", f"--user-data-dir={tmp}\\profile", "--hide-scrollbars",
                        f"--window-size={size}", f"--screenshot={png}", page.as_uri()], check=True, timeout=120,
                       capture_output=True)
        for _ in range(80):                      # the browser writes the file after its launcher has returned
            if png.exists() and png.stat().st_size > 0:
                break
            time.sleep(0.5)
        time.sleep(1.0)
    if not png.exists():
        raise SystemExit(f"no screenshot written: {png}")
    print(png)


def main() -> None:
    listing = ftp("UNLOAD_IMM.LS")
    alarms = ftp("ERRACT.LS")
    io = "\n".join(ln.rstrip() for ln in ftp("IOSTATE.DG").split("\n") if KEEP.match(ln))
    (RES / "virtual_controller_listing.ls").write_text(listing, encoding="utf-8", newline="\n")
    (RES / "virtual_controller_active_alarms.txt").write_text(alarms, encoding="utf-8", newline="\n")
    (RES / "virtual_controller_io_extract.txt").write_text(io + "\n", encoding="utf-8", newline="\n")
    who = "R-30iB Plus virtual controller, software V9.40, arm M-710iD/70"
    shot("UNLOAD_IMM as the controller lists it after translating the LS file",
         f"{who}. Text read back over FTP (md:UNLOAD_IMM.LS).", [("md:UNLOAD_IMM.LS", listing)],
         IMG / "vc_program_listing.png", "900,1700")
    shot("After the time-out path: active alarm and the signals of the test",
         f"{who}. Texts read over FTP; the I/O status file is cut down to the signals of this test, read after the "
         "run script had switched its outputs off.",
         [("md:ERRACT.LS (active alarms)", alarms), ("md:IOSTATE.DG, extract", io)], IMG / "vc_alarm_io.png", "900,720")


if __name__ == "__main__":
    main()

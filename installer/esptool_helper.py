"""
rdzTTGOsonde Flasher – esptool helpers (flash, read_flash, write_flash).

Runs esptool in a subprocess so the UI stays responsive; output is captured
and can be appended to a log callback.
"""

from __future__ import annotations

import subprocess
import sys
from typing import Callable, Optional


def _esptool_cmd(
    port: Optional[str],
    baud: str,
    extra: list[str],
    on_line: Optional[Callable[[str], None]] = None,
) -> tuple[bool, str]:
    """Run esptool with given args. If port is None or 'Auto', omit --port."""
    base = [
        sys.executable,
        "-m",
        "esptool",
        "--chip",
        "esp32",
        "--baud",
        baud,
        "--before",
        "default-reset",
        "--after",
        "hard-reset",
    ]
    if port and port != "Auto" and not port.startswith("("):
        base.extend(["--port", port])
    cmd = base + extra
    try:
        proc = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )
        out_lines = []
        for line in proc.stdout or []:
            # Keep \r so UI can detect "return to start of line" and overwrite progress line
            line_no_nl = line.rstrip("\n")
            out_lines.append(line_no_nl.rstrip("\r"))
            if on_line:
                if line_no_nl.endswith("\r"):
                    on_line(line_no_nl)
                else:
                    on_line(line_no_nl + "\n")
        proc.wait()
        return proc.returncode == 0, "\n".join(out_lines)
    except FileNotFoundError:
        msg = "esptool not found. Install with: pip install esptool"
        if on_line:
            on_line(msg + "\n")
        return False, msg
    except Exception as e:
        msg = str(e)
        if on_line:
            on_line(msg + "\n")
        return False, msg


def flash_firmware(
    port: Optional[str],
    baud: str,
    image_path: str,
    on_line: Optional[Callable[[str], None]] = None,
) -> tuple[bool, str]:
    """Write full image at 0x1000. image_path = path to .bin file."""
    return _esptool_cmd(
        port,
        baud,
        [
            "write-flash",
            "-z",
            "--flash-mode", "dio",
            "--flash-freq", "80m",
            "--flash-size", "detect",
            "0x1000",
            image_path,
        ],
        on_line=on_line,
    )


def read_backup(
    port: Optional[str],
    baud: str,
    out_path: str,
    on_line: Optional[Callable[[str], None]] = None,
) -> tuple[bool, str]:
    """Read flash 0x1000 size 0x3FF000 to out_path."""
    return _esptool_cmd(
        port,
        baud,
        ["read-flash", "0x1000", "0x3FF000", out_path],
        on_line=on_line,
    )


def write_backup(
    port: Optional[str],
    baud: str,
    image_path: str,
    on_line: Optional[Callable[[str], None]] = None,
) -> tuple[bool, str]:
    """Restore full backup: write image at 0x1000."""
    return _esptool_cmd(
        port,
        baud,
        [
            "write-flash",
            "-z",
            "--flash-mode", "dio",
            "--flash-freq", "80m",
            "--flash-size", "detect",
            "0x1000",
            image_path,
        ],
        on_line=on_line,
    )

"""
rdzTTGOsonde Flasher – esptool helpers (flash, read_flash, write_flash).

Runs esptool in a subprocess so the UI stays responsive; output is captured
and can be appended to a log callback.
"""

from __future__ import annotations

import subprocess
import sys
from typing import Callable, Optional

# Per-operation no-stub flags.
# Read operations (read-flash): _no_stub_read
# Write operations (write-flash, flash): _no_stub_write
# set_no_stub() sets both (used by CLI --no-stub).
_no_stub_read: bool = True
_no_stub_write: bool = False


def set_no_stub(value: bool) -> None:
    """Set --no-stub for ALL operations (read and write). Used by CLI --no-stub flag."""
    global _no_stub_read, _no_stub_write
    _no_stub_read = _no_stub_write = bool(value)


def set_no_stub_read(value: bool) -> None:
    """Set --no-stub for read operations only (read-flash, download_filesystem, read_backup)."""
    global _no_stub_read
    _no_stub_read = bool(value)


def set_no_stub_write(value: bool) -> None:
    """Set --no-stub for write operations only (write-flash, flash_firmware, upload_filesystem)."""
    global _no_stub_write
    _no_stub_write = bool(value)


def get_no_stub_read() -> bool:
    return _no_stub_read


def get_no_stub_write() -> bool:
    return _no_stub_write


def _esptool_cmd(
    port: Optional[str],
    baud: str,
    extra: list[str],
    on_line: Optional[Callable[[str], None]] = None,
    no_stub: bool = False,
) -> tuple[bool, str]:
    """Run esptool with given args. If port is None or 'Auto', omit --port.
    no_stub should be passed explicitly by each public function using _no_stub_read
    or _no_stub_write as appropriate.
    """
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
    if no_stub:
        base.append("--no-stub")
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


def read_flash_region(
    port: Optional[str],
    baud: str,
    offset: int,
    size: int,
    out_path: str,
    on_line: Optional[Callable[[str], None]] = None,
) -> tuple[bool, str]:
    """Read a region of flash to a file. offset/size in bytes."""
    return _esptool_cmd(
        port,
        baud,
        ["read-flash", "%#x" % offset, "%#x" % size, out_path],
        on_line=on_line,
        no_stub=_no_stub_read,
    )


def write_flash_region(
    port: Optional[str],
    baud: str,
    offset: int,
    size: int,
    image_path: str,
    on_line: Optional[Callable[[str], None]] = None,
) -> tuple[bool, str]:
    """Write a region of flash from a file. offset/size in bytes (e.g. 0x10000, 0x140000)."""
    return _esptool_cmd(
        port,
        baud,
        [
            "write-flash",
            "-z",
            "--flash-mode", "dio",
            "--flash-freq", "80m",
            "--flash-size", "detect",
            "%#x" % offset,
            image_path,
        ],
        on_line=on_line,
        no_stub=_no_stub_write,
    )


# The full flash image file (downloaded or backed up) starts at this flash address.
# esptool write-flash 0x1000 image.bin → file byte 0 lands at flash 0x1000.
# esptool read-flash  0x1000 ...       → backup byte 0 is flash 0x1000.
# To convert a flash address to a file offset: file_offset = flash_addr - FLASH_IMAGE_BASE
FLASH_IMAGE_BASE = 0x1000

# Partition offsets and sizes (flash addresses, partitions-rdz.csv)
PARTITION_APP0_OFFSET = 0x10000
PARTITION_APP0_SIZE = 0x140000
PARTITION_SPIFFS_OFFSET = 0x320000
PARTITION_SPIFFS_SIZE = 0xD0000

# Size of the full-flash backup region (0x1000 … end of usable flash)
BACKUP_SIZE = 0x3FF000


def download_filesystem(
    port: Optional[str],
    baud: str,
    out_path: str,
    on_line: Optional[Callable[[str], None]] = None,
) -> tuple[bool, str]:
    """Read the LittleFS/SPIFFS partition from device to a file."""
    return read_flash_region(port, baud, PARTITION_SPIFFS_OFFSET, PARTITION_SPIFFS_SIZE, out_path, on_line)


def upload_filesystem(
    port: Optional[str],
    baud: str,
    image_path: str,
    on_line: Optional[Callable[[str], None]] = None,
) -> tuple[bool, str]:
    """Write a LittleFS image file to the device's filesystem partition.

    Uses --flash-mode keep so esptool does NOT patch the image header bytes
    (which would corrupt the LittleFS superblock at offset 0 of the partition).
    """
    return _esptool_cmd(
        port,
        baud,
        [
            "write-flash",
            "--flash-mode", "keep",
            "--flash-size", "keep",
            "%#x" % PARTITION_SPIFFS_OFFSET,
            image_path,
        ],
        on_line=on_line,
        no_stub=_no_stub_write,
    )


def flash_app_partition(
    port: Optional[str],
    baud: str,
    full_image_path: str,
    on_line: Optional[Callable[[str], None]] = None,
) -> tuple[bool, str]:
    """
    Flash only the app partition (code update): slice from full image at app0 offset/size
    and write to 0x10000. full_image_path is a full flash image (e.g. from download or backup).

    The full image file byte 0 corresponds to flash address FLASH_IMAGE_BASE (0x1000),
    so the app0 partition (flash 0x10000) is at file offset 0x10000 - 0x1000 = 0xF000.
    """
    import tempfile
    file_offset = PARTITION_APP0_OFFSET - FLASH_IMAGE_BASE  # 0xF000
    try:
        with open(full_image_path, "rb") as f:
            f.seek(file_offset)
            chunk = f.read(PARTITION_APP0_SIZE)
        if len(chunk) < PARTITION_APP0_SIZE:
            # Pad with 0xFF to partition size
            chunk = chunk + b"\xff" * (PARTITION_APP0_SIZE - len(chunk))
        with tempfile.NamedTemporaryFile(delete=False, suffix=".bin") as tf:
            tf.write(chunk)
            tf_path = tf.name
        try:
            return write_flash_region(
                port, baud, PARTITION_APP0_OFFSET, PARTITION_APP0_SIZE, tf_path, on_line
            )
        finally:
            try:
                import os
                os.unlink(tf_path)
            except Exception:
                pass
    except FileNotFoundError:
        msg = "Image file not found: %s" % full_image_path
        if on_line:
            on_line(msg + "\n")
        return False, msg
    except Exception as e:
        msg = str(e)
        if on_line:
            on_line(msg + "\n")
        return False, msg


def read_partition_table(
    port: Optional[str],
    baud: str,
    on_line: Optional[Callable[[str], None]] = None,
) -> tuple[Optional[dict], Optional[str]]:
    """
    Read partition table from device (0x8000, 0x1000). Return (partitions_dict, error).
    partitions_dict: name -> (offset_int, size_int).
    """
    import tempfile
    import os
    with tempfile.NamedTemporaryFile(delete=False, suffix=".bin") as tf:
        tf_path = tf.name
    try:
        ok, out = _esptool_cmd(
            port,
            baud,
            ["read-flash", "0x8000", "0x1000", tf_path],
            on_line=on_line,
            no_stub=_no_stub_read,
        )
        if not ok:
            return None, out or "read-flash failed"
        with open(tf_path, "rb") as f:
            data = f.read()
        # Simple CSV parse would need gen_esp32part; for now return known default
        # so callers can use spiffs offset/size
        return {
            "app0": (PARTITION_APP0_OFFSET, PARTITION_APP0_SIZE),
            "spiffs": (PARTITION_SPIFFS_OFFSET, PARTITION_SPIFFS_SIZE),
        }, None
    finally:
        try:
            os.unlink(tf_path)
        except Exception:
            pass


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
        no_stub=_no_stub_write,
    )


def read_backup(
    port: Optional[str],
    baud: str,
    out_path: str,
    on_line: Optional[Callable[[str], None]] = None,
) -> tuple[bool, str]:
    """Read flash 0x1000 size BACKUP_SIZE to out_path."""
    return _esptool_cmd(
        port,
        baud,
        ["read-flash", "%#x" % FLASH_IMAGE_BASE, "%#x" % BACKUP_SIZE, out_path],
        on_line=on_line,
        no_stub=_no_stub_read,
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
        no_stub=_no_stub_write,
    )

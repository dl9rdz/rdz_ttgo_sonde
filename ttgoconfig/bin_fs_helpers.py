"""
Unified helpers to list and extract from .bin images that may contain
either LittleFS or SPIFFS. Auto-detect by trying LittleFS first, then SPIFFS.
Used by the GUI for "Extract filesystem from backup" and Wi-Fi tab .bin view.
"""

from __future__ import annotations

import os
from typing import List, Optional, Tuple

from . import littlefs_helper as lfs
from .esptool_helper import PARTITION_SPIFFS_OFFSET, PARTITION_SPIFFS_SIZE

LFS_BACKUP_BASE = getattr(lfs, "BACKUP_FLASH_BASE", 0x1000)


def list_fs_from_bin_auto(
    bin_path: str,
    file_offset: Optional[int] = None,
    partition_size: Optional[int] = None,
) -> Tuple[List[dict], str]:
    """
    List root files from a .bin that may be LittleFS or SPIFFS.
    Tries LittleFS first (with given or default partition offset/size), then SPIFFS.
    Returns (entries, fs_type) where fs_type is "littlefs" or "spiffs".
    entries: list of {"name", "size", "dir"}.
    """
    if file_offset is None and partition_size is None:
        try:
            sz = os.path.getsize(bin_path)
        except OSError:
            sz = 0
        if sz > PARTITION_SPIFFS_SIZE:
            file_offset = PARTITION_SPIFFS_OFFSET - LFS_BACKUP_BASE
            partition_size = PARTITION_SPIFFS_SIZE
        else:
            file_offset = 0
            partition_size = None

    # Try LittleFS first
    try:
        entries = lfs.list_fs_from_bin(bin_path, file_offset=file_offset or 0, partition_size=partition_size)
        if entries:
            return entries, "littlefs"
    except Exception:
        pass

    # Try SPIFFS (uses partition table or fallbacks)
    try:
        from .spiffs_reader import open_spiffs
        img = open_spiffs(bin_path)
        entries = img.list_files()
        if entries:
            return entries, "spiffs"
    except Exception:
        pass

    return [], "littlefs"  # default so callers can still try LittleFS read


def read_file_from_bin_auto(
    bin_path: str,
    name: str,
    fs_type: str,
    file_offset: Optional[int] = None,
    partition_size: Optional[int] = None,
) -> bytes:
    """Read one file from a .bin. fs_type from list_fs_from_bin_auto."""
    name = name.lstrip("/")
    if fs_type == "spiffs":
        from .spiffs_reader import open_spiffs
        img = open_spiffs(bin_path)
        content = img.read_file("/" + name if not name.startswith("/") else name)
        if content is None:
            raise FileNotFoundError("File not in image: %s" % name)
        return content
    # LittleFS
    return lfs.read_file_from_bin(
        bin_path, name, file_offset=file_offset or 0, partition_size=partition_size
    )


def extract_from_bin_auto(
    backup_path: str,
    dest_dir: str,
    file_offset: Optional[int] = None,
    partition_size: Optional[int] = None,
) -> Tuple[bool, Optional[str]]:
    """
    Extract filesystem from a backup .bin to dest_dir.
    Tries LittleFS first, then SPIFFS. Returns (True, None) or (False, error_msg).
    """
    if file_offset is None and partition_size is None:
        try:
            sz = os.path.getsize(backup_path)
        except OSError:
            return False, "Cannot read backup file"
        if sz > PARTITION_SPIFFS_SIZE:
            file_offset = PARTITION_SPIFFS_OFFSET - LFS_BACKUP_BASE
            partition_size = PARTITION_SPIFFS_SIZE
        else:
            file_offset = 0
            partition_size = None

    # Try LittleFS first (extract_from_backup expects flash_offset and size)
    flash_offset = (file_offset or 0) + LFS_BACKUP_BASE
    size = partition_size or 0
    if size <= 0:
        try:
            sz = os.path.getsize(backup_path)
            if sz > PARTITION_SPIFFS_SIZE:
                flash_offset = PARTITION_SPIFFS_OFFSET
                size = PARTITION_SPIFFS_SIZE
        except OSError:
            pass
    if size > 0:
        try:
            lfs.extract_from_backup(backup_path, dest_dir, flash_offset, size)
            return True, None
        except Exception:
            pass

    # Try SPIFFS
    try:
        from .spiffs_reader import open_spiffs
        img = open_spiffs(backup_path)
        entries = img.list_files()
        if not entries:
            return False, "No LittleFS or SPIFFS filesystem found in image"
        os.makedirs(dest_dir, exist_ok=True)
        for it in entries:
            name = (it.get("name") or "").lstrip("/")
            if not name or it.get("dir"):
                continue
            content = img.read_file(it["name"])
            if content is None:
                continue
            out_path = os.path.join(dest_dir, name)
            out_dir = os.path.dirname(out_path)
            if out_dir:
                os.makedirs(out_dir, exist_ok=True)
            with open(out_path, "wb") as f:
                f.write(content)
        return True, None
    except Exception as e:
        return False, str(e)

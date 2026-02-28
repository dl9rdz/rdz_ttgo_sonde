"""
Parse ESP32 binary partition table from a .bin image.

Partition table is at flash 0x8000, size 0x1000. In a full backup .bin where
file byte 0 = flash 0x1000, table is at file offset 0x7000.
Format: 32-byte entries (magic 0x50AA, type, subtype, offset, size, label[16], flags).
End marker: 32 bytes 0xFF. MD5 block may follow entries (0xEBEB magic); we stop at 0xFF.
"""

from __future__ import annotations

import struct
from typing import Dict, Optional, Tuple

# Flash offset and size of partition table
PARTITION_TABLE_FLASH_OFFSET = 0x8000
PARTITION_TABLE_SIZE = 0x1000

# Entry: magic(2) type(1) subtype(1) offset(4) size(4) label(16) flags(4) = 32
ENTRY_MAGIC = b"\xAA\x50"
ENTRY_FORMAT = "<2sBBLL16sL"
ENTRY_SIZE = 32

# First 2 bytes of MD5 block (skip this 32-byte block when scanning)
MD5_MAGIC = b"\xEB\xEB"


def parse_partition_table(data: bytes) -> Dict[str, Tuple[int, int]]:
    """
    Parse binary partition table. Returns dict name -> (flash_offset, size).
    Stops at 0xFF*32 or when data runs out. Skips 32-byte blocks that start with MD5_MAGIC.
    """
    result: Dict[str, Tuple[int, int]] = {}
    offset = 0
    while offset + ENTRY_SIZE <= len(data):
        chunk = data[offset : offset + ENTRY_SIZE]
        offset += ENTRY_SIZE
        if chunk == b"\xFF" * ENTRY_SIZE:
            break
        if chunk[:2] == MD5_MAGIC:
            continue
        if chunk[:2] != ENTRY_MAGIC:
            continue
        _magic, _type, _subtype, flash_offset, size, label_bytes, _flags = struct.unpack(
            ENTRY_FORMAT, chunk
        )
        name = label_bytes.split(b"\x00")[0].decode("utf-8", errors="replace").strip()
        if not name:
            continue
        result[name] = (flash_offset, size)
    return result


def get_partition_table_from_bin(
    bin_path: str,
    flash_base: int = 0x1000,
) -> Optional[Dict[str, Tuple[int, int]]]:
    """
    Read partition table from a .bin file. Assumes file byte 0 = flash_base
    (e.g. 0x1000 for full backup). Reads at file offset (0x8000 - flash_base),
    size 0x1000. Returns name -> (flash_offset, size) or None on read/parse error.
    """
    file_offset = PARTITION_TABLE_FLASH_OFFSET - flash_base
    if file_offset < 0:
        return None
    try:
        with open(bin_path, "rb") as f:
            f.seek(file_offset)
            data = f.read(PARTITION_TABLE_SIZE)
    except OSError:
        return None
    if len(data) < ENTRY_SIZE:
        return None
    return parse_partition_table(data)

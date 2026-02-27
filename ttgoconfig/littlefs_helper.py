"""
LittleFS image packing and unpacking helpers using the littlefs-python API.

Partition layout (partitions-rdz.csv):
  spiffs  0x320000  0xD0000  (block_count = 208 @ 4096 B blocks)
"""

from __future__ import annotations

import os
from typing import Union

from littlefs import LittleFS

BLOCK_SIZE = 4096


def _extract_recursive(fs: LittleFS, fs_dir: str, local_dir: str) -> None:
    """Walk fs_dir in the LittleFS image and write all files to local_dir."""
    os.makedirs(local_dir, exist_ok=True)
    for entry in fs.scandir(fs_dir):
        fs_path = fs_dir.rstrip("/") + "/" + entry.name
        local_path = os.path.join(local_dir, entry.name)
        if entry.type == 2:  # directory
            _extract_recursive(fs, fs_path, local_path)
        else:
            with fs.open(fs_path, "rb") as f_in:
                data = f_in.read()
            with open(local_path, "wb") as f_out:
                f_out.write(data)


def pack_directory(src_dir: str, image_path: str, block_count: int) -> None:
    """Walk src_dir and create a LittleFS v2.0 image at image_path.

    disk_version=0x00020000 forces the on-disk format to LittleFS v2.0,
    which is what the ESP32 Arduino/ESP-IDF LittleFS driver expects.
    Without this, newer littlefs-python versions default to v2.1.
    """
    fs = LittleFS(block_size=BLOCK_SIZE, block_count=block_count, disk_version=0x00020000)
    for root, dirs, files in os.walk(src_dir):
        rel_root = os.path.relpath(root, src_dir)
        if rel_root != ".":
            fs_dir = "/" + rel_root.replace(os.sep, "/")
            try:
                fs.mkdir(fs_dir)
            except Exception:
                pass
        for fname in files:
            local_path = os.path.join(root, fname)
            fs_path = "/" + os.path.relpath(local_path, src_dir).replace(os.sep, "/")
            with open(local_path, "rb") as f_in:
                content = f_in.read()
            with fs.open(fs_path, "wb") as f_out:
                f_out.write(content)
    with open(image_path, "wb") as f:
        f.write(fs.context.buffer)


def unpack_bytes(data: Union[bytes, bytearray], dest_dir: str, block_count: int) -> None:
    """Generic: mount a LittleFS image from raw bytes and extract all files to dest_dir."""
    fs = LittleFS(block_size=BLOCK_SIZE, block_count=block_count, mount=False)
    fs.context.buffer = bytearray(data)
    fs.mount()
    _extract_recursive(fs, "/", dest_dir)


def unpack_image(image_path: str, dest_dir: str, block_count: int) -> None:
    """Read a LittleFS image file and extract all files to dest_dir."""
    with open(image_path, "rb") as f:
        data = f.read()
    unpack_bytes(data, dest_dir, block_count)


BACKUP_FLASH_BASE = 0x1000  # read_backup reads starting at flash 0x1000; file byte 0 = flash 0x1000


def extract_from_backup(backup_path: str, dest_dir: str, flash_offset: int, size: int) -> None:
    """Slice a full backup .bin at the given flash_offset and extract the LittleFS filesystem.

    The backup file is produced by read_backup() which reads from flash 0x1000 onward,
    so backup byte 0 = flash 0x1000.  File offset = flash_offset - BACKUP_FLASH_BASE.
    """
    file_offset = flash_offset - BACKUP_FLASH_BASE
    with open(backup_path, "rb") as f:
        f.seek(file_offset)
        data = f.read(size)
    unpack_bytes(data, dest_dir, size // BLOCK_SIZE)

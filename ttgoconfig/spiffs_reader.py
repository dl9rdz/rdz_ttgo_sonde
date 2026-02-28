"""
Standalone pure-Python SPIFFS image reader.

Reads a SPIFFS filesystem image (e.g. from a full backup .bin or raw partition)
and supports listing files and extracting file contents. Layout follows
pellepl/spiffs with block_size=4096, page_size=256 (as used by mkspiffs in
scripts/makeimage.py).

Usage:
  - From code: SpiffsImage(path, file_offset=0, partition_size=None)
  - For full backup .bin (file byte 0 = flash 0x1000): use file_offset and
    partition_size for the SPIFFS partition (defaults below).
  - list_files() -> [{"name", "size", "dir"}, ...]
  - read_file(name) -> bytes

CLI: python -m ttgoconfig.spiffs_reader <image.bin> [file_offset] [partition_size]
     If only image.bin is given and file is large, assume full backup and use
     SPIFFS partition slice.
"""

from __future__ import annotations

import struct
import sys
from typing import Any, List, Optional

# Layout (matches mkspiffs -b 4096 -p 256)
BLOCK_SIZE = 4096
PAGE_SIZE = 256
PAGES_PER_BLOCK = BLOCK_SIZE // PAGE_SIZE  # 16
OBJ_ID_SIZE = 2
# Number of lookup pages per block: max(1, (PAGES_PER_BLOCK * OBJ_ID_SIZE) / PAGE_SIZE) = 1
OBJ_LOOKUP_PAGES = 1
# Data pages per block (indexable via lookup)
OBJ_LOOKUP_MAX_ENTRIES = PAGES_PER_BLOCK - OBJ_LOOKUP_PAGES  # 15

# Full backup .bin: file byte 0 = flash 0x1000
FLASH_IMAGE_BASE = 0x1000
# Known SPIFFS partition layouts (flash_offset, size). Try in order when auto-detecting.
PARTITION_LAYOUTS = [
    (0x320000, 0xD0000),   # partitions-rdz.csv (current)
    (0x290000, 0x170000),  # partitions-esp32v1 / older master images
    (0x290000, 0x160000),  # partitions-esp32v2.csv
]
# Default = first layout (for explicit offset/size)
PARTITION_SPIFFS_OFFSET = PARTITION_LAYOUTS[0][0]
PARTITION_SPIFFS_SIZE = PARTITION_LAYOUTS[0][1]
DEFAULT_FILE_OFFSET = PARTITION_SPIFFS_OFFSET - FLASH_IMAGE_BASE
DEFAULT_PARTITION_SIZE = PARTITION_SPIFFS_SIZE

# Page header (every data/index page)
PAGE_HEADER_SIZE = 5  # obj_id(2) + span_ix(2) + flags(1)
# Object index header: page_hdr(5) + align(3) + size(4) + type(1) + name(32) = 45
OBJ_NAME_LEN = 32
OBJ_IX_HEADER_ALIGN = 3  # after 5-byte header, align to 4
OBJ_IX_HEADER_PAYLOAD = 4 + 1 + OBJ_NAME_LEN  # size + type + name
OBJ_IX_HEADER_TOTAL = PAGE_HEADER_SIZE + OBJ_IX_HEADER_ALIGN + OBJ_IX_HEADER_PAYLOAD  # 45
OBJ_IX_ENTRY_SIZE = 2  # spiffs_page_ix
# Index entries in object index header page (rest of 256 bytes)
OBJ_IX_HEADER_ENTRIES = (PAGE_SIZE - OBJ_IX_HEADER_TOTAL) // OBJ_IX_ENTRY_SIZE  # 105
# Object index (continuation) page: header(5)+align(3)=8, then entries
OBJ_IX_PAGE_HEADER_SIZE = PAGE_HEADER_SIZE + OBJ_IX_HEADER_ALIGN  # 8
OBJ_IX_PAGE_ENTRIES = (PAGE_SIZE - OBJ_IX_PAGE_HEADER_SIZE) // OBJ_IX_ENTRY_SIZE  # 124
# Data page payload
DATA_PAGE_PAYLOAD_SIZE = PAGE_SIZE - PAGE_HEADER_SIZE  # 251

# Obj id in lookup: 0xFFFF = free, 0 = deleted, MSB set = index page
OBJ_ID_FREE = 0xFFFF
OBJ_ID_DELETED = 0x0000
OBJ_ID_IX_FLAG = 1 << 15

# Page header flags (C: "if 0, page is deleted" => bit set = valid/not deleted)
PH_FLAG_USED = 1 << 0
PH_FLAG_FINAL = 1 << 1
PH_FLAG_INDEX = 1 << 2
PH_FLAG_IXDELE = 1 << 6   # if 0, index header is being deleted
PH_FLAG_DELETED = 1 << 7  # if 0, page is deleted
SPIFFS_UNDEFINED_LEN = 0xFFFFFFFF

# Object types
SPIFFS_TYPE_FILE = 1
SPIFFS_TYPE_DIR = 2

def _parse_page_header(buf: bytes, offset: int = 0) -> tuple[int, int, int]:
    """Return (obj_id, span_ix, flags) from a 5-byte page header."""
    obj_id, span_ix, flags = struct.unpack_from("<HHB", buf, offset)
    return obj_id, span_ix, flags


def _parse_object_ix_header(buf: bytes, offset: int = 0) -> tuple[int, int, str]:
    """Parse object index header payload (after page header). offset points to size. Returns (size, type, name)."""
    size, otype = struct.unpack_from("<IB", buf, offset)
    if size == SPIFFS_UNDEFINED_LEN:
        size = 0
    name_bytes = buf[offset + 5 : offset + 5 + OBJ_NAME_LEN]
    name = name_bytes.split(b"\x00")[0].decode("utf-8", errors="replace")
    return size, otype, name


def _is_lookup_page(page_ix: int) -> bool:
    """True if page_ix is the first page of its block (lookup page)."""
    return (page_ix % PAGES_PER_BLOCK) < OBJ_LOOKUP_PAGES


def _block_for_page(page_ix: int) -> int:
    return page_ix // PAGES_PER_BLOCK


def _entry_to_page_ix(block_ix: int, entry_ix: int) -> int:
    """Convert (block, lookup entry index) to global page index. entry_ix 0..14."""
    return block_ix * PAGES_PER_BLOCK + OBJ_LOOKUP_PAGES + entry_ix


class SpiffsImage:
    """
    Read-only view of a SPIFFS image (from bytes or file slice).
    """

    def __init__(
        self,
        data: Optional[bytes] = None,
        path: Optional[str] = None,
        file_offset: int = 0,
        partition_size: Optional[int] = None,
    ) -> None:
        """
        Load from either bytes or file.
        - data: raw SPIFFS partition bytes (use this or path, not both).
        - path: path to .bin file.
        - file_offset: byte offset in file where SPIFFS partition starts (default 0).
        - partition_size: size of partition in bytes; if None and path given, read to EOF.
        """
        if data is not None and path is not None:
            raise ValueError("Provide either data or path, not both.")
        if data is not None:
            self._data = data
        else:
            if path is None:
                raise ValueError("Provide either data or path.")
            with open(path, "rb") as f:
                f.seek(file_offset)
                self._data = f.read(partition_size) if partition_size is not None else f.read()
        self._size = len(self._data)
        self._num_blocks = self._size // BLOCK_SIZE

    def _read_page(self, page_ix: int) -> bytes:
        """Read full page at page_ix (0-based)."""
        start = page_ix * PAGE_SIZE
        if start + PAGE_SIZE > self._size:
            raise IndexError("Page %d out of range" % page_ix)
        return self._data[start : start + PAGE_SIZE]

    def _read_lookup_entries(self, block_ix: int) -> List[int]:
        """Read object IDs from the lookup page of block block_ix. Returns list of 15 obj_ids.
        Some SPIFFS builds put the 15 obj_ids at block start (no header); others use a 5-byte
        page header first. Try start=0 first; if that yields any index-page id (MSB set), use it."""
        page_ix = block_ix * PAGES_PER_BLOCK
        page = self._read_page(page_ix)
        for start in (0, PAGE_HEADER_SIZE):
            entries = []
            for i in range(OBJ_LOOKUP_MAX_ENTRIES):
                off = start + i * OBJ_ID_SIZE
                if off + 2 > PAGE_SIZE:
                    break
                obj_id = struct.unpack_from("<H", page, off)[0]
                entries.append(obj_id)
            if len(entries) == OBJ_LOOKUP_MAX_ENTRIES and any(
                e not in (OBJ_ID_FREE, OBJ_ID_DELETED) and (e & OBJ_ID_IX_FLAG)
                for e in entries
            ):
                return entries
        # Fallback: header at 0, entries at 5
        return list(
            struct.unpack_from(
                "<%dH" % OBJ_LOOKUP_MAX_ENTRIES, page, PAGE_HEADER_SIZE
            )
        )

    def _find_object_index_headers(self) -> List[tuple[int, int, int, str, int]]:
        """Scan all blocks for object index header pages. Returns list of (obj_id, page_ix, size, name, type)."""
        result: List[tuple[int, int, int, str, int]] = []
        for block_ix in range(self._num_blocks):
            entries = self._read_lookup_entries(block_ix)
            for entry_ix, obj_id in enumerate(entries):
                if obj_id == OBJ_ID_FREE or obj_id == OBJ_ID_DELETED:
                    continue
                if not (obj_id & OBJ_ID_IX_FLAG):
                    continue
                page_ix = _entry_to_page_ix(block_ix, entry_ix)
                try:
                    page = self._read_page(page_ix)
                except IndexError:
                    continue
                oid, span_ix, flags = _parse_page_header(page, 0)
                if span_ix != 0:
                    continue
                # C spec: "if 0, page is deleted". Skip when both DELET and IXDELE are clear (truly deleted).
                if (flags & (PH_FLAG_DELETED | PH_FLAG_IXDELE)) == 0:
                    continue
                # Index page: USED=0, FINAL=0, INDEX=0 (bits 0-2 clear).
                lo = flags & 7
                if lo not in (0, 6, 7):
                    continue
                # Parse object index header payload (after 5-byte header + 3 align = 8)
                size, otype, name = _parse_object_ix_header(page, 8)
                result.append((oid & 0x7FFF, page_ix, size, name, otype))
        return result

    def list_files(self) -> List[dict[str, Any]]:
        """
        List all files and directories in the root. Returns list of dicts
        with "name", "size", "dir" (True for directories).
        Deduplicates by name (keeps entry with highest page_ix = latest copy), matching mkspiffs -l.
        """
        headers = self._find_object_index_headers()
        by_name: dict[str, tuple[int, int, str, int]] = {}
        for _oid, page_ix, size, name, otype in headers:
            if not name or name.startswith("\x00"):
                continue
            if name not in by_name or page_ix > by_name[name][0]:
                by_name[name] = (page_ix, size, name, otype)
        return [
            {"name": name, "size": size, "dir": otype == SPIFFS_TYPE_DIR}
            for _page_ix, size, name, otype in by_name.values()
        ]

    def _collect_data_page_indices(self, obj_id: int, header_page_ix: int) -> List[int]:
        """
        Collect all data page indices for this object in span order.
        obj_id is the data object id (no IX flag). Lookup may store either
        data id or index id (data_id | IX_FLAG); match both.
        """
        ix_obj_id = obj_id | OBJ_ID_IX_FLAG
        index_pages: List[tuple[int, int]] = []  # (span_ix, page_ix)
        for block_ix in range(self._num_blocks):
            entries = self._read_lookup_entries(block_ix)
            for entry_ix, oid in enumerate(entries):
                # Lookup can store raw obj_id, ix_obj_id, or byte-swapped (e.g. 0x0b00 for 11)
                if oid not in (ix_obj_id, obj_id, (obj_id << 8) | (obj_id >> 8)):
                    continue
                page_ix = _entry_to_page_ix(block_ix, entry_ix)
                try:
                    page = self._read_page(page_ix)
                except IndexError:
                    continue
                _oid, span_ix, flags = _parse_page_header(page, 0)
                if _oid != ix_obj_id:
                    continue
                lo = flags & 7
                if lo not in (0, 6, 7):
                    continue
                index_pages.append((span_ix, page_ix))
        # Keep latest page per span_ix (same object can appear in multiple blocks)
        by_span: dict[int, int] = {}
        for span_ix, page_ix in index_pages:
            if span_ix not in by_span or page_ix > by_span[span_ix]:
                by_span[span_ix] = page_ix
        index_pages = [(s, by_span[s]) for s in sorted(by_span.keys())]

        data_page_indices: List[int] = []
        for span_ix, page_ix in index_pages:
            page = self._read_page(page_ix)
            if span_ix == 0:
                # Header page: index entries start at OBJ_IX_HEADER_TOTAL
                n_entries = OBJ_IX_HEADER_ENTRIES
                off = OBJ_IX_HEADER_TOTAL
            else:
                # Continuation index page: entries start at OBJ_IX_PAGE_HEADER_SIZE
                n_entries = OBJ_IX_PAGE_ENTRIES
                off = OBJ_IX_PAGE_HEADER_SIZE
            for i in range(n_entries):
                if off + 2 > PAGE_SIZE:
                    break
                pix = struct.unpack_from("<H", page, off)[0]
                # 0xFFFF can mean free slot in index
                if pix != 0xFFFF and pix != 0:
                    data_page_indices.append(pix)
                off += 2
        return data_page_indices

    def read_file(self, name: str) -> Optional[bytes]:
        """
        Read entire file content by name. Returns None if not found or not a file.
        Uses the latest copy (highest page_ix) when multiple headers share the same name.
        """
        headers = self._find_object_index_headers()
        best: Optional[tuple[int, int, int]] = None
        for oid, header_page_ix, size, n, otype in headers:
            if n != name or otype != SPIFFS_TYPE_FILE:
                continue
            if best is None or header_page_ix > best[1]:
                best = (oid, header_page_ix, size)
        if best is None:
            return None
        oid, header_page_ix, size = best
        data_page_indices = self._collect_data_page_indices(oid, header_page_ix)
        chunks = []
        remaining = size
        for pix in data_page_indices:
            if remaining <= 0:
                break
            try:
                page = self._read_page(pix)
            except IndexError:
                continue
            payload = page[PAGE_HEADER_SIZE:PAGE_HEADER_SIZE + DATA_PAGE_PAYLOAD_SIZE]
            take = min(len(payload), remaining)
            chunks.append(payload[:take])
            remaining -= take
        return b"".join(chunks)


def get_spiffs_slice_for_bin(
    path: str,
    file_offset: Optional[int] = None,
    partition_size: Optional[int] = None,
) -> tuple[int, Optional[int], dict]:
    """
    Determine (file_offset, partition_size) for the SPIFFS partition in a .bin file.
    Returns (file_off, size, info). info has: partition_table_found, source,
    flash_offset, flash_size, file_offset, partition_size (for verbose output).
    File offset = flash offset - FLASH_IMAGE_BASE when .bin starts at flash 0x1000.
    """
    info: dict = {
        "partition_table_found": False,
        "source": "explicit",
        "flash_offset": None,
        "flash_size": None,
        "file_offset": None,
        "partition_size": None,
    }
    with open(path, "rb") as f:
        f.seek(0, 2)
        size = f.tell()

    if file_offset is not None or partition_size is not None:
        off = file_offset or 0
        info["file_offset"] = off
        info["partition_size"] = partition_size
        info["source"] = "explicit"
        if partition_size is not None:
            info["flash_offset"] = off + FLASH_IMAGE_BASE
            info["flash_size"] = partition_size
        return off, partition_size, info

    # Try partition table (full backup: table at flash 0x8000 → file offset 0x7000)
    try:
        from .partition_table import get_partition_table_from_bin
        table = get_partition_table_from_bin(path, flash_base=FLASH_IMAGE_BASE)
        info["partition_table_found"] = table is not None
        if table and "spiffs" in table:
            flash_off, part_size = table["spiffs"]
            off = flash_off - FLASH_IMAGE_BASE
            if off >= 0 and size >= off + part_size:
                info["source"] = "partition_table"
                info["flash_offset"] = flash_off
                info["flash_size"] = part_size
                info["file_offset"] = off
                info["partition_size"] = part_size
                return off, part_size, info
    except Exception:
        info["partition_table_found"] = False

    # Fallback: try known layouts
    for flash_off, part_size in PARTITION_LAYOUTS:
        off = flash_off - FLASH_IMAGE_BASE
        if off < 0 or size < off + part_size:
            continue
        img = SpiffsImage(path=path, file_offset=off, partition_size=part_size)
        if img.list_files():
            info["source"] = "fallback_layout"
            info["flash_offset"] = flash_off
            info["flash_size"] = part_size
            info["file_offset"] = off
            info["partition_size"] = part_size
            return off, part_size, info

    # Last resort: first layout or whole file
    if size >= DEFAULT_FILE_OFFSET + DEFAULT_PARTITION_SIZE:
        off, part_size = DEFAULT_FILE_OFFSET, DEFAULT_PARTITION_SIZE
        info["source"] = "default_layout"
        info["flash_offset"] = PARTITION_SPIFFS_OFFSET
        info["flash_size"] = part_size
        info["file_offset"] = off
        info["partition_size"] = part_size
        return off, part_size, info
    info["source"] = "whole_file"
    info["file_offset"] = 0
    info["partition_size"] = None
    return 0, None, info


def open_spiffs(
    path: str,
    file_offset: Optional[int] = None,
    partition_size: Optional[int] = None,
) -> SpiffsImage:
    """
    Open a SPIFFS image from a file. If file_offset and partition_size are None
    and the file is large enough to be a full backup: first reads the partition
    table at flash 0x8000 and uses the "spiffs" partition offset/size if present;
    otherwise tries known layouts and returns the first that yields files.
    File offset in .bin = flash offset - FLASH_IMAGE_BASE (0x1000).
    """
    off, part_size, _ = get_spiffs_slice_for_bin(path, file_offset, partition_size)
    return SpiffsImage(path=path, file_offset=off, partition_size=part_size)


def _debug_dump(img: SpiffsImage) -> None:
    """Print raw block 0 lookup and first data page headers for debugging."""
    print("DEBUG: partition size %d bytes, %d blocks" % (img._size, img._num_blocks))
    print("DEBUG: block 0 lookup page (first 32 bytes): %s" % (
        " ".join("%02x" % b for b in img._data[:32])))
    entries = img._read_lookup_entries(0)
    print("DEBUG: block 0 lookup entries (15): %s" % (
        " ".join("0x%04x" % e for e in entries)))
    for entry_ix, obj_id in enumerate(entries):
        if obj_id == OBJ_ID_FREE or obj_id == OBJ_ID_DELETED:
            continue
        page_ix = _entry_to_page_ix(0, entry_ix)
        try:
            page = img._read_page(page_ix)
        except IndexError:
            continue
        oid, span_ix, flags = _parse_page_header(page, 0)
        print("DEBUG: page_ix=%d obj_id=0x%04x span_ix=%d flags=0x%02x" % (
            page_ix, oid, span_ix, flags))
        if span_ix == 0 and not (flags & PH_FLAG_DELETED):
            try:
                size, otype, name = _parse_object_ix_header(page, 8)
                print("DEBUG:   -> name=%r size=%d type=%d" % (name, size, otype))
            except Exception as ex:
                print("DEBUG:   -> parse err: %s" % ex)


def main() -> None:
    args = [a for a in sys.argv[1:] if a != "--debug"]
    debug = len(args) != len(sys.argv[1:])
    if len(args) < 1:
        print("Usage: python -m ttgoconfig.spiffs_reader [--debug] <image.bin> [file_offset] [partition_size]")
        print("  image.bin: path to full backup .bin (from 0x1000) or raw SPIFFS partition")
        print("  If only image.bin given and file is large, uses partition table or known layouts.")
        print("  --debug: dump block 0 lookup and page headers.")
        sys.exit(1)
    path = args[0]
    file_offset_arg: Optional[int] = None
    partition_size_arg: Optional[int] = None
    if len(args) >= 2:
        file_offset_arg = int(args[1], 0)
    if len(args) >= 3:
        partition_size_arg = int(args[2], 0)

    try:
        file_off, part_size, info = get_spiffs_slice_for_bin(path, file_offset_arg, partition_size_arg)
    except Exception as e:
        print("Error opening image: %s" % e, file=sys.stderr)
        sys.exit(2)

    # Verbose: partition table and offsets
    print("Partition table: %s" % ("yes" if info.get("partition_table_found") else "no"))
    print("Source: %s" % info.get("source", "?"))
    if info.get("flash_offset") is not None and info.get("flash_size") is not None:
        print("Flash:  offset 0x%x  size 0x%x (%d bytes)" % (
            info["flash_offset"], info["flash_size"], info["flash_size"]))
    if info.get("file_offset") is not None:
        print("File:   offset 0x%x  size %s" % (
            info["file_offset"],
            "0x%x (%d bytes)" % (info["partition_size"], info["partition_size"])
            if info.get("partition_size") is not None else "to EOF"))
    print()

    try:
        img = SpiffsImage(path=path, file_offset=file_off, partition_size=part_size)
    except Exception as e:
        print("Error reading SPIFFS: %s" % e, file=sys.stderr)
        sys.exit(2)

    if debug:
        _debug_dump(img)
        print()

    entries = img.list_files()
    print("Files (%d):" % len(entries))
    for e in entries:
        kind = "dir" if e["dir"] else "file"
        print("  %s  %s  %s  %s" % (kind, e["name"], e["size"], "(dir)" if e["dir"] else ""))
    if not entries:
        print("  (none or not a valid SPIFFS image)")

    # Optional: extract first file as test
    for e in entries:
        if not e["dir"] and e["size"] and e["size"] < 10000:
            content = img.read_file(e["name"])
            if content is not None:
                print("\nFirst 200 bytes of %s:" % e["name"])
                print(repr(content[:200]))
            break


if __name__ == "__main__":
    main()

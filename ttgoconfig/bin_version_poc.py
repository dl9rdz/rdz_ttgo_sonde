#!/usr/bin/env python3
"""
Proof-of-concept: read .bin file(s) and print embedded version string.

Usage: python bin_version_poc.py file1.bin [file2.bin ...]

Searches each file for version-like strings matching:
  - mainN.NN*           (e.g. main1.0, main2.34)
  - master vN.N.N*     (e.g. master v0.7.0)
  - master_vN.N.N*     (e.g. master_v0.7.1)
  - devYYYYMM*         (e.g. dev202602, dev20260221)
  - develYYYYMM*       (e.g. devel202602)

Prints one line per file: "filename: version" or "filename: " if none found.
"""

from __future__ import annotations

import re
import sys

# Max bytes to scan per file (app partition is ~1.25 MB; avoid reading huge dumps)
MAX_SCAN_BYTES = 2 * 1024 * 1024

# Order of patterns: first match wins. Trailing \w* allows optional suffix (e.g. dev20240905b).
VERSION_PATTERNS = [
    re.compile(rb"main\d+\.\d+(?:\.\d+)*\w*"),                # mainN.NN* + optional suffix
    re.compile(rb"master[\s_]v\d+\.\d+(?:\.\d+)*\w*"),        # master v0.7.0 or master_v0.7.1 + optional
    re.compile(rb"dev\d{4}\d{2}\d*\w*"),                      # devYYYYMM* + optional (e.g. dev20240905b)
    re.compile(rb"devel\d{4}\d{2}\d*\w*"),                     # develYYYYMM* + optional
]


def find_version(data: bytes) -> str | None:
    """Search binary data for a version string. Returns first match or None."""
    for pat in VERSION_PATTERNS:
        m = pat.search(data)
        if m:
            try:
                return m.group(0).decode("ascii")
            except UnicodeDecodeError:
                continue
    return None


# Full backup .bin: file byte 0 = flash 0x1000; app0 at flash 0x10000 → file offset 0xF000, size 0x140000
APP_PARTITION_FILE_OFFSET = 0xF000
APP_PARTITION_SIZE = 0x140000


def get_version_from_bin(bin_path: str) -> str | None:
    """
    Read the app partition from a .bin file and search for a version string.
    For full backup images: read from file offset 0xF000, size 0x140000.
    For smaller files (e.g. raw app image): read from start up to app partition size.
    Returns the first matched version string or None.
    """
    try:
        with open(bin_path, "rb") as f:
            f.seek(0, 2)
            size = f.tell()
            if size >= APP_PARTITION_FILE_OFFSET + APP_PARTITION_SIZE:
                f.seek(APP_PARTITION_FILE_OFFSET)
                data = f.read(APP_PARTITION_SIZE)
            else:
                f.seek(0)
                data = f.read(min(size, APP_PARTITION_SIZE))
    except OSError:
        return None
    return find_version(data)


def main() -> None:
    if len(sys.argv) < 2:
        print("Usage: python bin_version_poc.py file1.bin [file2.bin ...]", file=sys.stderr)
        sys.exit(1)

    for path in sys.argv[1:]:
        try:
            with open(path, "rb") as f:
                data = f.read(MAX_SCAN_BYTES)
        except OSError as e:
            print(f"{path}: (error: {e})")
            continue

        version = find_version(data)
        if version:
            print(f"{path}: {version}")
        else:
            print(f"{path}: ")


if __name__ == "__main__":
    main()

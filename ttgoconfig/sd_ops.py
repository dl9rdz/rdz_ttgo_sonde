"""
rdzTTGOsonde Flasher – SD card over Wi‑Fi (files.json, sd/ path).

Shared by GUI and CLI. Uses same session/auth as wifi_ops.
"""

from __future__ import annotations

import os
import urllib.parse
from typing import Any, List, Optional, Tuple


def list_dir(
    session: Any, base_url: str, dir_path: str = ""
) -> Tuple[List[dict], Optional[str]]:
    """
    GET /files.json in pages with optional dir and start=...
    Return (all_entries, error_msg).
    """
    base = base_url.rstrip("/") + "/files.json"
    all_entries: List[dict] = []
    start = 0

    while True:
        params = {"start": str(start)}
        if dir_path:
            params["dir"] = dir_path
        query = urllib.parse.urlencode(params, safe="/")
        url = base + "?" + query
        try:
            r = session.get(url, timeout=8)
            if r.status_code != 200:
                return [], "HTTP %s – SD card may not be available." % r.status_code
            data = r.json()
        except Exception as e:
            return [], str(e)
        if not isinstance(data, list):
            return [], "Invalid files.json response."
        if not data:
            break
        all_entries.extend(data)
        start += len(data)

    return all_entries, None


def collect_files_recursive(
    session: Any, base_url: str, dir_path: str
) -> List[str]:
    """Return list of SD paths (file or dir/file) for all files under dir_path."""
    entries, err = list_dir(session, base_url, dir_path)
    if err:
        return []
    out: List[str] = []
    for item in entries:
        name = item.get("name", "")
        if not name:
            continue
        full = (dir_path + "/" + name).strip("/") if dir_path else name
        if item.get("dir"):
            out.extend(collect_files_recursive(session, base_url, full))
        else:
            out.append(full)
    return out


def fetch_paths_to_dir(
    session: Any,
    base_url: str,
    paths: List[str],
    outdir: str,
    progress_cb: Optional[Any] = None,
) -> Tuple[int, List[str]]:
    """
    Download each path via GET base_url + "sd/" + path into outdir.
    progress_cb(current_index, total) optional.
    Returns (saved_count, list of error strings).
    """
    saved = 0
    errors: List[str] = []
    total = len(paths)
    for i, path in enumerate(paths):
        try:
            r = session.get(base_url + "sd/" + urllib.parse.quote(path), timeout=15)
            if r.status_code != 200:
                errors.append("%s: HTTP %s" % (path, r.status_code))
                if progress_cb and total:
                    progress_cb(i + 1, total)
                continue
            local = os.path.join(outdir, path)
            os.makedirs(os.path.dirname(local) or ".", exist_ok=True)
            with open(local, "wb") as f:
                f.write(r.content)
            saved += 1
        except Exception as e:
            errors.append("%s: %s" % (path, e))
        if progress_cb and total:
            progress_cb(i + 1, total)
    return saved, errors


def resolve_sd_path_to_list(
    session: Any, base_url: str, file_or_dir: str
) -> Tuple[List[str], Optional[str]]:
    """
    Resolve file_or_dir to a list of SD paths to fetch.
    If it names a file, return [path]. If it names a directory, return all files under it.
    """
    parent = os.path.dirname(file_or_dir) or ""
    name = os.path.basename(file_or_dir)
    entries, err = list_dir(session, base_url, parent)
    if err:
        return [], err
    for item in entries:
        if item.get("name") == name:
            if item.get("dir"):
                paths = collect_files_recursive(session, base_url, file_or_dir)
                return paths, None
            return [file_or_dir], None
    return [], "Path not found: %s" % file_or_dir

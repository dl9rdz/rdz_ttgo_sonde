"""
rdzTTGOsonde Flasher – Firmware fetch and resolve (manifest, download.html).

Shared by GUI and CLI for downloading firmware from website:
resolve main/dev2 from manifest, fetch older list, pattern match, download to temp.
"""

from __future__ import annotations

import fnmatch
import json
import os
import re
import tempfile
import urllib.request
from html.parser import HTMLParser
from typing import Callable, Dict, List, Optional, Tuple


def fetch_manifest_firmware_url(
    base_url: str, choice: str
) -> Tuple[Optional[str], Optional[str]]:
    """
    Return (firmware_url, error_msg).
    choice is 'Stable (main)', 'Development (dev2)', 'main', or 'dev2'.
    Uses manifest.json builds: first main → main, first dev → dev2.
    """
    base = base_url.rstrip("/")
    manifest_url = base + "/manifest.json"
    try:
        req = urllib.request.Request(manifest_url)
        with urllib.request.urlopen(req, timeout=15) as r:
            data = json.loads(r.read().decode("utf-8"))
    except Exception as e:
        return None, "Failed to fetch manifest: %s" % e
    builds = data.get("builds") or []
    want_main = "main" in choice.lower() or "Stable" in choice
    for b in builds:
        fw = (b.get("fwversion") or "").strip()
        if want_main and fw.startswith("main"):
            return _first_part_url(base, b)
        if not want_main and (fw.startswith("dev") or "dev" in choice.lower()):
            return _first_part_url(base, b)
    return None, "No matching build in manifest (Stable/main or Development/dev2)."


def _first_part_url(base: str, build: dict) -> Tuple[Optional[str], Optional[str]]:
    parts = build.get("parts") or []
    if not parts:
        return None, "Manifest build has no parts."
    path = (parts[0].get("path") or "").strip()
    if not path:
        return None, "Manifest part has no path."
    return base + "/" + path.lstrip("/"), None


def _manifest_code_part_url(base: str, build: dict) -> Optional[str]:
    """Return URL for 'code' part if present (e.g. update.bin at 0x10000)."""
    for part in build.get("parts") or []:
        label = (part.get("label") or part.get("type") or "").lower()
        if "code" in label or "update" in label or "app" in label:
            path = (part.get("path") or "").strip()
            if path:
                return base + "/" + path.lstrip("/")
    return None


class _DownloadTableParser(HTMLParser):
    """Extract (version, href) pairs from any table row that contains both a <code>
    element (version string) and an <a href="...-full.bin"> link.
    No restriction on table class or id.  Nested tables are handled via a depth counter.
    """

    def __init__(self) -> None:
        super().__init__()
        self.rows: List[Tuple[str, str]] = []
        self._table_depth = 0   # incremented for every <table>, decremented for </table>
        self._in_row = False
        self._current_code = ""
        self._current_href = ""
        self._in_code = False

    def handle_starttag(self, tag: str, attrs: list) -> None:
        if tag == "table":
            self._table_depth += 1
        if self._table_depth > 0 and tag == "tr":
            self._in_row = True
            self._current_code = ""
            self._current_href = ""
        if self._in_row and tag == "code":
            self._in_code = True
        if self._in_row and tag == "a":
            for k, v in attrs:
                if k == "href" and v and v.endswith("-full.bin"):
                    self._current_href = v
                    break

    def handle_endtag(self, tag: str) -> None:
        if tag == "table":
            self._table_depth = max(0, self._table_depth - 1)
        if tag == "tr" and self._in_row:
            if self._current_code and self._current_href:
                self.rows.append((self._current_code.strip(), self._current_href))
            self._in_row = False
        if tag == "code":
            self._in_code = False

    def handle_data(self, data: str) -> None:
        if self._in_code:
            self._current_code += data


def fetch_download_html_versions(
    base_url: str,
) -> Tuple[List[Tuple[str, str]], Optional[str]]:
    """
    Fetch download.html and parse (display_name, full_url) for full-image rows.
    Return ([(display, url)], error_msg).
    """
    url = base_url.rstrip("/") + "/download.html"
    try:
        req = urllib.request.Request(url)
        with urllib.request.urlopen(req, timeout=15) as r:
            html = r.read().decode("utf-8", errors="replace")
    except Exception as e:
        return [], "Failed to fetch download.html: %s" % e
    parser = _DownloadTableParser()
    try:
        parser.feed(html)
    except Exception as e:
        return [], "Failed to parse download.html: %s" % e
    base = base_url.rstrip("/")
    seen: Dict[str, int] = {}
    result: List[Tuple[str, str]] = []
    for version, href in parser.rows:
        full_url = base + "/" + href.lstrip("/")
        display = version
        if display in seen:
            seen[display] += 1
            display = "%s (%d)" % (version, seen[display])
        else:
            seen[version] = 1
        result.append((display, full_url))
    return result, None


def list_available_versions(base_url: str) -> Tuple[List[Tuple[str, str]], Optional[str]]:
    """
    Return ([(display_name, url)], error) for all available firmware versions.
    Includes "Stable (main)" and "Development (dev2)" from manifest, then older from download.html.
    """
    default = []
    # Manifest: main and dev2
    for label, choice in [("Stable (main)", "main"), ("Development (dev2)", "dev2")]:
        url, err = fetch_manifest_firmware_url(base_url, choice)
        if not err and url:
            default.append((label, url))
    older, err = fetch_download_html_versions(base_url)
    if err:
        return default, None  # still return manifest pair
    return default + older, None


def pattern_match_version(
    name: str, versions: List[Tuple[str, str]]
) -> Optional[Tuple[str, str]]:
    """
    Match name against version list. name can be:
    - exact display string
    - fnmatch pattern (e.g. main*, dev*)
    Return (display, url) if single match, else None.
    """
    exact = []
    pattern_matches = []
    name_lower = name.lower()
    for display, url in versions:
        if display == name:
            exact.append((display, url))
        if fnmatch.fnmatch(display.lower(), name_lower) or name_lower in display.lower():
            pattern_matches.append((display, url))
    if exact:
        return exact[0]
    if len(pattern_matches) == 1:
        return pattern_matches[0]
    return None


def resolve_firmware_url(
    base_url: str, name: str, fetched_versions: Optional[Dict[str, str]] = None
) -> Tuple[Optional[str], Optional[str]]:
    """
    Resolve name to a firmware URL.
    - If name is 'main' or 'dev2': use manifest.
    - Else if fetched_versions and name in it: return that URL.
    - Else fetch older list and pattern-match name; single match → URL.
    Return (url, error_msg).
    """
    if name.lower() in ("main", "dev2"):
        return fetch_manifest_firmware_url(base_url, name)
    if fetched_versions and name in fetched_versions:
        return fetched_versions[name], None
    versions, err = list_available_versions(base_url)
    if err:
        return None, err
    match = pattern_match_version(name, versions)
    if match:
        return match[1], None
    return None, "No unique match for %r. Available: %s" % (
        name,
        ", ".join(d for d, _ in versions[:15]),
    )


def download_firmware_to_temp(
    url: str, on_progress: Optional[Callable[[int, int], None]] = None
) -> Tuple[Optional[str], Optional[str]]:
    """
    Download url to a temporary .bin file. Return (path, error_msg).
    Caller must unlink the temp file when done.
    """
    try:
        req = urllib.request.Request(url)
        with urllib.request.urlopen(req, timeout=120) as r:
            data = r.read()
        tf = tempfile.NamedTemporaryFile(delete=False, suffix=".bin")
        tf.write(data)
        tf.close()
        return tf.name, None
    except Exception as e:
        return None, "Download failed: %s" % e

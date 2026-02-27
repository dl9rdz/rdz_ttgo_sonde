"""
rdzTTGOsonde Flasher – Wi‑Fi HTTP ops (session, login, get/put files).

Shared by GUI and CLI. Set names: config, qrg, wifi, screens, allconfig, all.
"""

from __future__ import annotations

import hashlib
import re
import socket
from typing import Any, List, Optional, Tuple

# Set name → list of filenames (align with scripts/ttgoconfig.py and GUI)
SCREENS = ["screens%d.txt" % i for i in range(1, 6)]
FILE_SETS = {
    "config": ["config.txt"],
    "qrg": ["qrg.txt"],
    "wifi": ["networks.txt"],
    "screens": SCREENS.copy(),
    "allconfig": ["config.txt", "qrg.txt", "networks.txt"],
    "all": ["config.txt", "qrg.txt", "networks.txt"] + SCREENS,
}


def resolve_host(host: str) -> Tuple[Optional[str], Optional[str]]:
    """Resolve host (IP or mDNS) to IP. Return (ip, error_msg)."""
    host = (host or "").strip()
    if not host:
        return None, "Host is empty."
    if "://" in host:
        host = host.split("://")[-1].split("/")[0].split(":")[0]
    try:
        ip = socket.gethostbyname(host)
        return ip, None
    except Exception as e:
        return None, str(e)


def base_url_for_host(host: str) -> Tuple[str, Optional[str]]:
    """Return (base_url, error). base_url is http://IP/."""
    ip, err = resolve_host(host)
    if err:
        return "", err
    return "http://%s/" % ip, None


def login(session: Any, base_url: str, user: str, password: str) -> Optional[str]:
    """
    Challenge-response login: GET login.html, POST auth.
    Returns None on success, error message on failure.
    """
    try:
        r = session.get(base_url + "login.html", timeout=10)
        r.raise_for_status()
        m = re.search(r'name="preauth"\s+value="([^"]+)"', r.text)
        if not m:
            return "Could not get login form (preauth)."
        preauth = m.group(1)
        auth_hex = hashlib.sha256(
            ("%s:%s:%s" % (user, preauth, password)).encode()
        ).hexdigest()
        r2 = session.post(
            base_url + "login.html",
            data={"user": user, "preauth": preauth, "auth": auth_hex},
            allow_redirects=True,
            timeout=10,
        )
        if r2.status_code == 401:
            return "Invalid credentials or login failed."
        return None
    except Exception as e:
        return "Login failed: %s" % e


def session_for_host(
    host: str,
    user: Optional[str] = None,
    password: Optional[str] = None,
) -> Tuple[Any, str, Optional[str]]:
    """
    Create requests session and optionally log in.
    Return (session, base_url, error_msg). base_url is http://IP/.
    """
    try:
        import requests
    except ImportError:
        return None, "", "requests not installed"
    base_url, err = base_url_for_host(host)
    if err:
        return None, "", err
    session = requests.Session()
    if user and password:
        err = login(session, base_url, user.strip(), password)
        if err:
            return None, base_url, err
    return session, base_url, None


def get_file(
    session: Any, base_url: str, name: str
) -> Tuple[Optional[bytes], Optional[str]]:
    """GET file from device. Return (content, error_msg)."""
    url = base_url + "file/" + name
    try:
        r = session.get(url, timeout=10)
        if r.status_code == 401:
            return None, "Permission denied (401). Use --user and --pass."
        if r.status_code == 404:
            return None, "Not found (404): %s" % name
        if r.status_code != 200:
            return None, "HTTP %s: %s" % (r.status_code, name)
        return r.content, None
    except Exception as e:
        return None, str(e)


def put_file(
    session: Any, base_url: str, filepath: str, remote_name: Optional[str] = None
) -> Optional[str]:
    """POST file to device. remote_name defaults to basename(filepath). Returns error_msg or None."""
    import os

    name = remote_name or os.path.basename(filepath)
    url = base_url + "file"
    try:
        with open(filepath, "rb") as f:
            files = {"data": (name, f)}
            r = session.post(url, files=files, timeout=30)
        if r.status_code == 401:
            return "Permission denied (401). Use --user and --pass."
        if r.status_code != 200:
            return "HTTP %s for %s" % (r.status_code, name)
        return None
    except FileNotFoundError:
        return "File not found: %s" % filepath
    except Exception as e:
        return str(e)


def get_files_by_kind(
    session: Any, base_url: str, setname: str
) -> Tuple[List[str], Optional[str]]:
    """Get list of filenames for set name. Return (filenames, error)."""
    if setname not in FILE_SETS:
        return [], "Unknown set: %s. Use one of: %s" % (
            setname,
            ", ".join(FILE_SETS.keys()),
        )
    return FILE_SETS[setname], None


def test_connection(host: str) -> Tuple[bool, str]:
    """GET status.json. Return (ok, message)."""
    session, base_url, err = session_for_host(host)
    if err:
        return False, err
    url = base_url.rstrip("/") + "/status.json"
    try:
        r = session.get(url, timeout=5)
        if r.status_code != 200:
            return False, "HTTP %s" % r.status_code
        body = r.text[:500] if len(r.text) > 500 else r.text
        return True, body
    except Exception as e:
        return False, str(e)

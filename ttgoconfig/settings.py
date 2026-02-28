"""
rdzTTGOsonde Flasher – Settings persistence.

Load/save to OS-appropriate config path:
- Windows: %LOCALAPPDATA%/rdzTTGOsonde/settings.json
- macOS/Linux: ~/.config/rdzTTGOsonde/settings.json
"""

from __future__ import annotations

import json
import os
from pathlib import Path
from typing import Any

DEFAULT_BACKUP_FOLDER = "~/rdzTTGOsonde/backups"
DEFAULT_DOWNLOAD_URL = "https://rdzsonde.org"
DEFAULT_BAUD = "921600"
DEFAULT_BAUD_READ = "115200"
DEFAULT_BAUD_WRITE = "921600"
DEFAULT_SERIAL_LOG_LINES = 10000


def _config_dir() -> Path:
    if os.name == "nt":
        base = os.environ.get("LOCALAPPDATA", os.path.expanduser("~"))
        return Path(base) / "rdzTTGOsonde"
    return Path.home() / ".config" / "rdzTTGOsonde"


def _settings_path() -> Path:
    return _config_dir() / "settings.json"


def _expand_path(p: str) -> str:
    return os.path.expanduser(os.path.expandvars(p))


def load() -> dict[str, Any]:
    path = _settings_path()
    if not path.is_file():
        return _defaults()
    try:
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
        out = _defaults()
        for k, v in data.items():
            if k in out:
                out[k] = v
        return out
    except Exception:
        return _defaults()


def _defaults() -> dict[str, Any]:
    return {
        "backup_folder": DEFAULT_BACKUP_FOLDER,
        "download_url": DEFAULT_DOWNLOAD_URL,
        "baud_rate": DEFAULT_BAUD_READ,
        "remember_port": True,
        "last_port": "",
        "serial_log_buffer_lines": DEFAULT_SERIAL_LOG_LINES,
        "strip_ansi_when_saving": False,
        "no_stub": False,
        "baud_rate_read": DEFAULT_BAUD_READ,
        "baud_rate_write": DEFAULT_BAUD_WRITE,
        "no_stub_read": False,
        "no_stub_write": False,
        "last_directory": "",
        "wifi_host": "rdzsonde.local",
        "wifi_user": "",
        "wifi_pass": "",
        "wifi_save_password": False,
    }


def save(data: dict[str, Any]) -> None:
    path = _settings_path()
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2)


def get_backup_folder_expanded(settings: dict[str, Any]) -> str:
    return _expand_path(settings.get("backup_folder", DEFAULT_BACKUP_FOLDER))

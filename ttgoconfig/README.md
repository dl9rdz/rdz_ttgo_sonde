# rdzTTGOsonde Desktop Flasher

Cross-platform desktop UI and CLI for flashing and managing rdzTTGOsonde firmware.

## Run the GUI

Use a virtual environment (recommended on macOS/Linux if system Python is externally managed). **Install dependencies first** with the same Python you use to run the app:

```bash
# From repo root
pip install -r ttgoconfig/requirements.txt
python -m ttgoconfig.app
```

Or from the ttgoconfig folder:

```bash
cd ttgoconfig
python3 -m venv .venv
source .venv/bin/activate   # Windows: .venv\Scripts\activate
pip install -r requirements.txt
python -m ttgoconfig.app
```

## Run the CLI

From repo root (so the `ttgoconfig` package is on the path):

```bash
python -m ttgoconfig.cli --help
python -m ttgoconfig.cli flash --install          # list available firmware versions
python -m ttgoconfig.cli flash --download main    # download and flash Stable (main)
python -m ttgoconfig.cli backup --port /dev/cu.usbserial-0001
python -m ttgoconfig.cli wifi get --kind all --dir ./out
python -m ttgoconfig.cli sd list
```

See `python -m ttgoconfig.cli --help` and per-command help (e.g. `python -m ttgoconfig.cli flash --help`) for all options. The legacy script `scripts/ttgoconfig.py` is a wrapper that runs the CLI; prefer `python -m ttgoconfig.cli` directly.

## Top-level modes

| Segment | Content (mockup) |
|--------|-------------------|
| **Flash** | Port dropdown + Refresh, firmware source (Download / Local file / From backup), Flash button, log area |
| **USB** | Port dropdown + Refresh, action buttons (Make backup, Restore backup, Restore selected files, Extract/Upload filesystem, Upload single file) |
| **Wi-Fi** | Host entry + Resolve, Test connection; User/Password; Trigger OTA, Restore files, Restore from backup .bin |
| **Serial** | Port dropdown, large scrollback log, Save to file (with “Strip ANSI” checkbox), Clear, Copy |
| **Settings** | Backup folder, Download server URL, Serial (baud, remember port), Serial log buffer size |

Port dropdown uses `pyserial`’s `list_ports` if available; otherwise shows a mock list.

## Next steps

- Wire Flash tab to esptool (port selection, download/local/backup, progress).
- Wire USB tab to backup/restore and filesystem extract/upload.
- Wire Wi-Fi tab to HTTP + OTA and file restore.
- Wire Serial tab to pyserial read thread and save/clear/copy.
- Persist Settings and last-used mode.

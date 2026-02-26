# rdzTTGOsonde Desktop Flasher (mockup)

Cross-platform desktop UI for flashing and managing rdzTTGOsonde firmware. This folder holds the first **mockup**: full GUI layout, no real flashing/serial/HTTP yet.

## Run the mockup

Use a virtual environment (recommended on macOS/Linux if system Python is externally managed). **Install dependencies first** with the same Python you use to run the app:

```bash
# From repo root
pip install -r installer/requirements.txt
python -m installer.app
```

Or from the installer folder:

```bash
cd installer
python3 -m venv .venv
source .venv/bin/activate   # Windows: .venv\Scripts\activate
pip install -r requirements.txt
python -m installer.app
```

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

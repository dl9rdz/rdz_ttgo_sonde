# ttgoconfig – Application Specification

**Version:** Current implementation (February 2026)  
**Target device:** rdzTTGOsonde (ESP32-based radiosonde receiver)  
**Platform:** macOS, Linux, Windows (Python 3.10+)

---

## Table of Contents

1. [Overview](#1-overview)
2. [Architecture](#2-architecture)
3. [High-Level Functional Requirements](#3-high-level-functional-requirements)
   - 3.1 Flash tab
   - 3.2 Serial tab
   - 3.3 USB tab
   - 3.4 Wi-Fi tab
   - 3.5 SD (via Wi-Fi) tab
   - 3.6 Settings tab
4. [CLI Reference](#4-cli-reference)
5. [Low-Level Implementation Details](#5-low-level-implementation-details)
   - 5.1 Module structure
   - 5.2 Settings persistence
   - 5.3 esptool integration
   - 5.4 Firmware resolution and download
   - 5.5 LittleFS image handling
   - 5.6 Wi-Fi HTTP protocol
   - 5.7 SD card protocol
   - 5.8 GUI concurrency model
   - 5.9 ANSI escape code handling
6. [Partition Layout](#6-partition-layout)
7. [File Sets](#7-file-sets)
8. [Known Limitations and Future Work](#8-known-limitations-and-future-work)

---

## 1. Overview

`ttgoconfig` is a cross-platform desktop application and CLI tool for managing rdzTTGOsonde firmware. It provides:

- **GUI**: a CustomTkinter desktop app with top-level navigation tabs.
- **CLI**: an `argparse`-based command-line interface mirroring all GUI functionality.
- Both share backend logic via a set of Python modules.

Entry points:
- **GUI**: `python -m ttgoconfig.app`
- **CLI**: `python -m ttgoconfig.cli <subcommand> [options]`

---

## 2. Architecture

```
ttgoconfig/
├── app.py            # CustomTkinter GUI (main window, all tab frames, ImprovDialog)
├── cli.py            # argparse CLI, one cmd_* function per subcommand
├── __main__.py       # Delegates to cli.main() for `python -m ttgoconfig`
├── esptool_helper.py # subprocess wrappers around esptool
├── firmware.py       # Firmware manifest/download.html fetch and resolution
├── wifi_ops.py       # HTTP session, login, get/put file, list_files (files.json?dir=.int), file sets
├── sd_ops.py         # SD card directory listing and file fetch over Wi-Fi
├── littlefs_helper.py# LittleFS image pack / unpack (littlefs-python API)
├── bin_fs_helpers.py # Auto-detect LittleFS vs SPIFFS in .bin; list/read/extract
├── bin_version_poc.py# Version string detection in app partition of .bin files
├── partition_table.py# Parse ESP32 partition table from .bin
├── spiffs_reader.py  # Pure-Python SPIFFS image list/read
├── improv_serial.py  # IMPROV serial protocol (packets, RPC get_info/scan/send_wifi)
├── settings.py       # Load/save JSON settings, defaults
└── requirements.txt  # customtkinter, pyserial, esptool, requests, littlefs-python
```

**Shared-module pattern**: `firmware.py`, `wifi_ops.py`, `sd_ops.py`, `littlefs_helper.py`, and `esptool_helper.py` are imported by both `app.py` and `cli.py`. GUI callbacks always run background work in threads and update widgets via `self.after(0, ...)`.

---

## 3. High-Level Functional Requirements

### 3.1 Flash Tab

#### FR-F1: Firmware source selection
The Flash tab shall offer three firmware sources selectable via a segmented button:
- **Download from website** – fetch from the configured download server.
- **Local file** – browse and select a local `.bin` file.
- **From backup** – select from `.bin` files found in the configured backup folder. The combo shows each filename with a detected version string in parentheses (e.g. `backup.bin (main 1.0.5)`) when available.

#### FR-F2: Flash type selection
A combo box labelled "What to flash" shall offer:
- **Full image** (default): write the entire firmware image file at flash address `0x1000`
  (i.e. `esptool write-flash 0x1000 image.bin`). File byte 0 maps to flash address `0x1000`.
- **Code update**: extract only the application partition (`app0`) from the image and write it
  at flash address `0x10000`.

**File offset calculation for code update:**
The full image file starts at flash `0x1000` (= `FLASH_IMAGE_BASE`). All partition flash
addresses must be converted to file offsets before slicing:

```
file_offset = flash_address - FLASH_IMAGE_BASE
```

For app0: flash address `0x10000`, so file offset = `0x10000 - 0x1000 = 0xF000`.
The slice is `0x140000` bytes long. This slice is written to flash at `0x10000`.

#### FR-F3: Download from website – stable and development
When source is "Download from website" and the selection is "Stable (main)" or "Development (dev2)", firmware URL is resolved from `manifest.json` at `{download_url}/manifest.json`. The first build whose `fwversion` starts with `main` is "Stable"; the first starting with `dev` is "Development".

#### FR-F4: Download from website – older versions
A button **"Fetch list of older versions"** (placed inline, to the right of the version combo box) fetches `{download_url}/download.html`, parses **any `<table>`** (no class or id restriction) and collects every `<tr>` row that contains both a `<code>` element (used as the version display name) and an `<a href="...-full.bin">` link (used as the firmware URL). Nested tables are handled correctly via a depth counter. The combo box is populated with the discovered versions. The button is disabled while loading and re-enabled on completion or error.

#### FR-F5: URL-resolved download
For an older version selected from the fetched list, the full URL stored from the fetch is used directly (bypassing manifest). For "Stable (main)" / "Development (dev2)", manifest lookup is used.

#### FR-F6: Flash execution
On clicking "Flash":
1. Firmware is obtained (downloaded to a temporary file, or read from local path / backup).
2. The "Flash" button is disabled; a log area shows real-time esptool output.
3. On success: "Flashed successfully." is shown in a messagebox.
4. On failure: error message is shown in a messagebox.
5. The temporary file (if downloaded) is deleted in a `finally` block.

#### FR-F7: Port selection
The port is selected via a combo box populated with available serial ports; the same selection is shared across Flash, USB, and Serial tabs. A "Refresh" button repopulates the list; if the saved port (from last app close) is in the new list, it is selected automatically. The last-used port is restored on startup and saved on app close.

#### FR-F8: Log output
Flash output from esptool is displayed in a scrollable text area with ANSI color support and progress-line overwriting (see §5.9).

---

### 3.2 Serial Tab

#### FR-S1: Serial connection
The user selects a port and baud rate. Clicking "Connect" opens the port and starts streaming output to a scrollable log area. Clicking "Disconnect" closes it.

#### FR-S2: Baud rate
A combo box offers baud rates: `115200`, `9600`, `19200`, `38400`, `57600`, `230400`, `460800`, `921600`. The initial value is restored from settings (`baud_rate`); if unset or invalid, it defaults to `115200`. The selected value is saved to settings when the app closes. There is no separate "Serial console baud" control in the Settings tab.

#### FR-S3: ANSI color rendering
Serial output is rendered with ANSI SGR color tags (foreground colors `30`–`37`, `90`–`97`; reset on `0`).

#### FR-S4: Save to file
A "Save to file" button writes the current log buffer to a user-selected file (ANSI escape sequences are preserved). The bottom row order is: Setup via IMPROV | Save to file | Clear | Copy.

#### FR-S5: Clear and copy
"Clear" erases the log. "Copy" copies the full log text to the clipboard.

#### FR-S5b: Setup via IMPROV
A "Setup via IMPROV" button is placed in the bottom row (left of "Save to file"). It opens a modal dialog that requests device info (name/version), runs a WiFi scan, and lets the user select a network (or enter a custom SSID) and password to provision the device. The dialog uses the IMPROV serial protocol (`improv_serial` module); the serial port is used exclusively during the flow (Serial tab disconnects if connected).

#### FR-S6: Buffer limit
The log is trimmed to a configurable maximum number of lines (default 10,000, configurable in Settings).

#### FR-S7: Thread safety
Serial reading runs in a background thread. Data is passed to the main thread via a `queue.Queue`. A periodic `after(200, _drain_serial_queue)` call drains the queue and appends to the text widget in batches.

---

### 3.3 USB Tab

All USB operations require a serial port (shared port combo with Flash tab). A scrollable log area at the bottom shows esptool output.

#### FR-U1: Make full backup
Prompts for a save path (default: `backup-YYYYMMDD-HHMM.bin` in the configured backup folder). Reads flash region `0x1000` size `0x3FF000` via `esptool read-flash` using the configured **read baud rate** (default `921600`). Uses the **no-stub (read)** setting (checked by default). Shows progress in the log.

#### FR-U2: Restore full backup
Prompts to select a `.bin` backup file and shows a confirmation dialog. Writes the image at `0x1000` via `esptool write-flash -z --flash-mode dio --flash-freq 80m --flash-size detect`.

#### FR-U3: Extract filesystem (from device)
Prompts for a destination directory. Reads the LittleFS partition (`0x320000`, `0xD0000`) via `esptool read-flash` at the configured read baud rate (default `921600`) to a temporary file. Unpacks the LittleFS image into the destination directory. Deletes the temporary file.

#### FR-U4: Upload filesystem (to device)
Prompts for a source directory and shows a confirmation dialog. Packs the directory into a LittleFS v2.0 image (block size 4096, block count 208). Flashes the image at `0x320000` via `esptool write-flash --flash-mode keep --flash-size keep`. Deletes the temporary image file.

#### FR-U5: Extract filesystem from backup image
Prompts for a `.bin` backup file and a destination directory. The implementation auto-detects LittleFS vs SPIFFS in the image (partition table or known offsets). For LittleFS, the filesystem partition is read and unpacked; for SPIFFS (older images), the SPIFFS image is read and files are extracted. No esptool or device needed.

---

### 3.4 Wi-Fi Tab

The Wi-Fi tab provides a dual-pane file manager: left = local folder or .bin image contents, right = device files. Host and optional auth are shared with the SD (via Wi-Fi) tab and persisted.

#### FR-W1: Host and auth
Row 1: Host entry (hostname or IP), resolved IP label, "Resolve", "Test connection". Row 2: "Auth (optional):" with User, Password, and "Save password" checkbox. Host, user, and (if checked) password are stored in settings and shared with the SD tab. Password is only written to config when "Save password" is checked; unchecking and closing removes it from the config file.

#### FR-W2: Resolve and test
"Resolve" resolves the hostname via `socket.gethostbyname()` and shows the IP. "Test connection" GETs `http://{ip}/status.json` and shows the result in the status line.

#### FR-W3: Left pane – local folder or .bin
- **Folder mode:** Shows current directory path (abbreviated when long; full path on hover). Tree lists "..", then directories, then files. Double-click ".." or a directory navigates. Files and dirs are sorted by extension then name (dirs by name).
- **Change:** Opens directory picker; selected folder becomes the current path.
- **Open .bin:** Opens file picker for `.bin`; switches to bin mode and lists the root of the LittleFS image (full backup or raw partition). ".." returns to folder view.
- In **bin mode**, "Download" is disabled; "Upload" uploads selected files from the .bin to the device (read from image, temp file, PUT, delete temp).

#### FR-W4: Right pane – device files
"Refresh" fetches the file list via GET `files.json?dir=.int`. If the device returns a non-empty list, that data is shown (names, sizes, dirs). If the response is empty or the request fails (e.g. old firmware), the static default list is shown and the status line displays: "Listing not supported by device firmware; showing default file names." Tree shows files (and directories when present) sorted by extension then name. Selection is used for "Download selected".

#### FR-W5: Sync actions (button row below panes)
- **Change** / **Open .bin** / **Upload to device →** / **← Download** / **Refresh** in one row (equal width).
- **Upload:** Selected files from the left (folder or bin) are uploaded to the device via POST `file`. Status line shows "Uploading N…" then "Uploading n/N…" then result.
- **Download:** Selected files from the right are downloaded to the current left-pane folder via GET `file/{name}`. Status line shows "Downloading N…", "Downloading n/N…", then result. Local tree is refreshed after success.

#### FR-W6: Status line
Full-width status line below the buttons shows connection status, errors, and progress (e.g. "Downloading 2/5…").

---

### 3.5 SD (via Wi-Fi) Tab

#### FR-SD1: Refresh / list
"Refresh" connects to the device, resolves the host, and GETs `/files.json` with pagination (optional `?dir=<path>`) to list SD card contents. Results are shown in a `ttk.Treeview` with Name and Size columns. While loading, the button is disabled and a placeholder shows status.

#### FR-SD2: Directory navigation
A ".." row is shown when not at root; double-click ".." or a directory navigates (fetches `/files.json?dir=<path>`). Column headers (Name, Size, Date) are clickable to sort; ".." stays first. Default sort is by date (newest first).

#### FR-SD3: Download selected
"Download selected" downloads the selected file(s) via GET `http://{ip}/sd/{path}` into a user-selected local directory.

#### FR-SD4: Download all
"Download all" recursively collects all files under the current SD directory and downloads them all, preserving directory structure.

#### FR-SD5: Progress bar
A `CTkProgressBar` shows progress during multi-file downloads (updated per file: `current/total`).

#### FR-SD6: Error handling
If host resolution fails, the placeholder shows an error message. The refresh button is always re-enabled after success or failure.

---

### 3.6 Settings Tab

Settings are loaded on startup and saved to disk when the user switches away from the Settings tab or when the app closes.

#### FR-SET1: Backup folder
Text entry for the backup directory path. "Browse…" opens a directory picker. Default: `~/rdzTTGOsonde/backups`.

#### FR-SET2: Download server URL
Text entry for the firmware download base URL. Default: `https://rdzsonde.org`.

#### FR-SET3: Serial tab baud rate
The Serial tab has a baud combo (no separate control in the Settings tab). Values: `115200`, `9600`, `19200`, `38400`, `57600`, `230400`, `460800`, `921600`. When no settings file exists, the default is `115200`. The selected value is saved to settings on app close.

#### FR-SET4: Serial log scroll-back buffer
Integer entry for maximum log lines. Default: 10,000.

#### FR-SET5: esptool read settings
Under the "esptool (advanced) → Read (backup, download FS)" sub-section:
- **Baud rate:** Combo box `921600` (default), `115200`, `460800`. Used for `read-flash` operations (Make backup, Extract filesystem from device).
- **Use --no-stub for read operations:** Checkbox (checked by default). When enabled, `--no-stub` is appended to all read-flash esptool calls. Needed for some ESP32-C3/S3 variants.

#### FR-SET5b: esptool write settings
Under the "esptool (advanced) → Write (flash, restore, upload FS)" sub-section:
- **Baud rate:** Combo box `921600` (default), `460800`, `115200`. Used for `write-flash` operations (Flash firmware, Restore backup, Upload filesystem).
- **Use --no-stub for write operations:** Checkbox. When enabled, `--no-stub` is appended to all write-flash esptool calls.

#### FR-SET6: Last port persistence
The last-used serial port is always saved to settings on app close and restored on startup. There is no UI option to disable this.

---

## 4. CLI Reference

All subcommands share global flags:

| Flag | Description |
|------|-------------|
| `--port PORT` | Serial port path |
| `--ttgo HOST` | Device hostname or IP (default: `rdzsonde.local`) |
| `--user U` | HTTP auth username |
| `--pass P` | HTTP auth password |
| `--baud RATE` | Override baud rate |
| `--dir DIR` | Directory for backup output or get/put |
| `--file FILE` | File path override |
| `--outdir DIR` | Output directory for fetch |
| `--no-stub` | Pass `--no-stub` to all esptool calls |

### 4.1 flash

```
ttgoconfig flash [--port PORT] [--baud RATE] [--part full|code]
                 [--download NAME | --file PATH | --install | NAME]
```

| Argument | Behavior |
|----------|----------|
| `--download main` or `dev2` | Resolve URL from manifest; download; flash |
| `--download NAME` | Fetch version list; pattern-match NAME; download; flash |
| `--install` | Print available version list only, no flash |
| `--file PATH` | Flash local file directly |
| `NAME` (positional) | If file exists: flash it. If `main`/`dev2`: use manifest. Else: pattern-match remote list |
| `--part code` | Flash app partition only (slice at `0x10000`, size `0x140000`) |
| `--part full` | Flash full image at `0x1000` (default) |

### 4.2 serial

```
ttgoconfig serial [BAUD] [--port PORT] [--save FILE] [--stripansi]
```

Streams serial output to stdout. `--save FILE` continuously appends raw bytes to FILE. `--stripansi` removes ANSI escape sequences before printing and saving. Exits on Ctrl+C.

### 4.3 backup

```
ttgoconfig backup [--port PORT] [--file PATH] [--dir DIR]
```

Reads flash `0x1000`–`0x3FF000` (full 4 MB image) at the configured read baud rate (default `921600`; override with `--baud`). Output file defaults to `backup-YYYYMMDD-HHMM.bin` in the configured backup folder or `--dir`.

### 4.4 uploadfs

```
ttgoconfig uploadfs [--port PORT] DIRECTORY [--keep-image]
```

Packs DIRECTORY into a LittleFS v2.0 image (208 blocks × 4096 B) and flashes it to `0x320000`. `--keep-image` retains the generated `.bin` after flashing (for inspection).

### 4.5 downloadfs

```
ttgoconfig downloadfs [--port PORT] DIRECTORY [--keep-image]
```

Reads `0xD0000` bytes at `0x320000` from device into a temporary file, then extracts the LittleFS v2.0 image to DIRECTORY. `--keep-image` retains the raw flash dump on failure (always retained on error; optionally retained on success).

### 4.6 extractfs

```
ttgoconfig extractfs DIRECTORY --image BACKUP.bin
```

Slices bytes at `0x320000` for `0xD0000` bytes from the backup file (no device needed) and extracts the LittleFS contents to DIRECTORY.

### 4.7 wifi

```
ttgoconfig wifi [--ttgo HOST] [--user U] [--pass P] [--dir DIR]
                get|put|test [FILENAME] [--kind SETNAME]
```

| Action | Behavior |
|--------|----------|
| `test` | GET `/status.json`; print response body |
| `get FILENAME` | Download `/file/FILENAME` to current directory (or `--dir`) |
| `get --kind SETNAME` | Download all files in the named set |
| `put FILENAME` | POST local file to `/file` on device |
| `put --kind SETNAME` | POST all files in the named set from current directory |

### 4.8 sd

```
ttgoconfig sd [--ttgo HOST] [--user U] [--pass P] list [DIR]
ttgoconfig sd [--ttgo HOST] [--user U] [--pass P] fetch PATH [--outdir DIR]
```

| Action | Behavior |
|--------|----------|
| `list [DIR]` | GET `/files.json?dir=DIR`; print entries |
| `fetch PATH` | Download file or recursively download directory to `--outdir` (default: cwd) |

---

## 5. Low-Level Implementation Details

### 5.1 Module Structure

| Module | Key exports |
|--------|-------------|
| `esptool_helper` | `_esptool_cmd`, `flash_firmware`, `flash_app_partition`, `read_backup`, `write_backup`, `read_flash_region`, `write_flash_region`, `download_filesystem`, `upload_filesystem`, `set_no_stub`, `set_no_stub_read`, `set_no_stub_write`, `PARTITION_*` constants |
| `firmware` | `fetch_manifest_firmware_url`, `fetch_download_html_versions`, `list_available_versions`, `resolve_firmware_url`, `download_firmware_to_temp` |
| `wifi_ops` | `FILE_SETS`, `resolve_host`, `session_for_host`, `login`, `get_file`, `put_file`, `list_files`, `get_files_by_kind`, `test_connection` |
| `sd_ops` | `list_dir`, `collect_files_recursive`, `resolve_sd_path_to_list`, `fetch_paths_to_dir` |
| `littlefs_helper` | `pack_directory`, `unpack_bytes`, `unpack_image`, `extract_from_backup`, `list_fs_from_bin`, `read_file_from_bin`, `BLOCK_SIZE`, `BACKUP_FLASH_BASE` |
| `bin_fs_helpers` | `list_fs_from_bin_auto`, `read_file_from_bin_auto`, `extract_from_bin_auto` (LittleFS/SPIFFS auto-detect) |
| `bin_version_poc` | `get_version_from_bin` (version string in app partition) |
| `partition_table` | `get_partition_table_from_bin`, `parse_partition_table` |
| `spiffs_reader` | `open_spiffs`, `SpiffsImage` (list/read SPIFFS images) |
| `improv_serial` | `build_packet`, `parse_packet`, `rpc_get_info`, `rpc_wifi_scan`, `rpc_send_wifi`, etc. |
| `settings` | `load`, `save`, `_defaults`, `get_backup_folder_expanded` |

### 5.2 Settings Persistence

**File location:**
- Windows: `%LOCALAPPDATA%/rdzTTGOsonde/settings.json`
- macOS/Linux: `~/.config/rdzTTGOsonde/settings.json`

**Schema:**

| Key | Type | Default | Description |
|-----|------|---------|-------------|
| `backup_folder` | str | `~/rdzTTGOsonde/backups` | Backup output directory |
| `download_url` | str | `https://rdzsonde.org` | Firmware download server base URL |
| `baud_rate` | str | `115200` | Serial tab baud (saved on app close from Serial combo; no control in Settings tab) |
| `baud_rate_read` | str | `921600` | esptool baud for read-flash operations (backup, download FS) |
| `baud_rate_write` | str | `921600` | esptool baud for write-flash operations (flash, restore, upload FS) |
| `remember_port` | bool | `true` | Legacy key; always true |
| `last_port` | str | `""` | Last-used serial port (shared across Flash/USB/Serial; saved on close, restored on startup; Refresh selects it when it becomes available) |
| `serial_log_buffer_lines` | int | `10000` | Serial log scroll-back limit |
| `strip_ansi_when_saving` | bool | `false` | Strip ANSI when saving serial log |
| `no_stub` | bool | `false` | Legacy: pass `--no-stub` to all esptool calls (CLI `--no-stub` flag sets both) |
| `no_stub_read` | bool | `true` | Pass `--no-stub` to esptool read operations (default on for best compatibility) |
| `no_stub_write` | bool | `false` | Pass `--no-stub` to esptool write operations |
| `last_directory` | str | `""` | Last-used directory for save/extract dialogs |
| `wifi_host` | str | `rdzsonde.local` | Wi-Fi/SD tab host (shared) |
| `wifi_user` | str | `""` | Wi-Fi/SD tab auth user (shared) |
| `wifi_pass` | str | `""` | Wi-Fi/SD tab auth password; only saved when `wifi_save_password` is true |
| `wifi_save_password` | bool | `false` | When true, password is persisted; when false, password is not written and any existing saved password is removed on next save |

Load merges saved keys into defaults so new keys survive upgrades. On save, the full merged dict is written.

In the GUI, `settings_mod.save()` is called:
- When the user switches away from the Settings tab (`_on_mode_change`).
- When the app window is closed (`_on_close`).
- When the browse-backup button is clicked (immediate save).
- When `_save_from_ui` is called from SettingsFrame.

In the CLI, settings are loaded once at startup in `main()` to apply `no_stub` fallback. The `--no-stub` flag always takes precedence over the saved setting.

### 5.3 esptool Integration

All esptool calls go through `_esptool_cmd(port, baud, extra, on_line)`.

**Command construction:**
```python
[sys.executable, "-m", "esptool",
 "--chip", "esp32",
 "--baud", baud,
 "--before", "default-reset",
 "--after", "hard-reset",
 # "--no-stub"  (if _no_stub is True)
 # "--port", port  (if port is not None/Auto)
] + extra
```

**Output streaming:** `subprocess.Popen` with `stdout=PIPE, stderr=STDOUT, text=True, bufsize=1`. Each line is passed to `on_line` callback. Lines ending in `\r` (carriage return without newline) are passed as-is so the GUI can implement progress-line overwriting. Lines ending in `\n` have the newline appended.

**Return value:** `(bool, str)` — `(success, combined_output)`.

**no-stub flags:** Two module-level booleans: `_no_stub_read` (for read operations, default `True`) and `_no_stub_write` (for write operations, default `False`). Each public function explicitly passes the appropriate flag to `_esptool_cmd` via its `no_stub` parameter:
- Read functions (`read_flash_region`, `read_backup`, `download_filesystem`, `read_partition_table`) pass `no_stub=_no_stub_read`.
- Write functions (`write_flash_region`, `write_backup`, `upload_filesystem`, `flash_firmware`) pass `no_stub=_no_stub_write`.
- `flash_app_partition` calls `write_flash_region` which uses `_no_stub_write`.

Setters:
- `set_no_stub_read(value)` — sets `_no_stub_read` (GUI settings for read operations).
- `set_no_stub_write(value)` — sets `_no_stub_write` (GUI settings for write operations).
- `set_no_stub(value)` — sets **both** (used by CLI `--no-stub` flag which applies to all operations).

At GUI startup, `set_no_stub_read` and `set_no_stub_write` are called from saved `no_stub_read` / `no_stub_write` settings. At CLI startup, `set_no_stub(True)` is called if `--no-stub` is provided or if the legacy `no_stub` setting is true.

**Individual operations:**

| Function | esptool command | Notes |
|----------|----------------|-------|
| `flash_firmware` | `write-flash -z --flash-mode dio --flash-freq 80m --flash-size detect 0x1000 <path>` | Full image |
| `write_backup` | `write-flash -z --flash-mode dio --flash-freq 80m --flash-size detect 0x1000 <path>` | Same as flash_firmware |
| `read_backup` | `read-flash 0x1000 0x3FF000 <path>` | Uses caller-supplied baud (CLI: `--baud`; GUI: read baud setting; default `921600`). Read operations use `--no-stub` by default in GUI. |
| `flash_app_partition` | Slice from image at **file offset `0xF000`** (`= 0x10000 - 0x1000`) for `0x140000` bytes, then `write-flash` at flash `0x10000` | Temp file, cleaned up |
| `write_flash_region` | `write-flash -z --flash-mode dio --flash-freq 80m --flash-size detect <offset> <path>` | Generic region write |
| `read_flash_region` | `read-flash <offset> <size> <path>` | Generic region read |
| `upload_filesystem` | `write-flash --flash-mode keep --flash-size keep 0x320000 <path>` | **No** flash-param patching |
| `download_filesystem` | `read-flash 0x320000 0xD0000 <path>` | |

**Full image file layout:** The full firmware image file (downloaded or backed up) has its byte 0 at flash address `0x1000` (`FLASH_IMAGE_BASE`). To convert any flash address to a file offset: `file_offset = flash_address - 0x1000`. Concretely: app0 (flash `0x10000`) is at file offset `0xF000`; SPIFFS (flash `0x320000`) is at file offset `0x31F000`. This applies both to downloaded firmware images (written with `write-flash 0x1000`) and to backup files (created with `read-flash 0x1000`).

**Critical:** `upload_filesystem` uses `--flash-mode keep --flash-size keep` (not `dio`/`80m`/`detect`) because the `dio`/`80m` options cause esptool to patch bytes 2–3 of the image (the flash parameter bytes of the ESP firmware image format). This patching corrupts the LittleFS superblock which begins at offset 0 of the partition image.

### 5.4 Firmware Resolution and Download

**Manifest approach** (for `main` and `dev2`):
1. GET `{base_url}/manifest.json` (timeout 15s).
2. Parse `builds[]` array. Each build has `fwversion` and `parts[]`.
3. First build with `fwversion` starting with `main` → Stable. First starting with `dev` → Development.
4. Return `base_url + "/" + parts[0].path`.

**download.html approach** (for older versions):
1. GET `{base_url}/download.html` (timeout 15s).
2. Parse with `html.parser` using `_DownloadTableParser`.
3. Any `<table>` is entered (no class/id restriction). A depth counter tracks nesting so nested tables are handled correctly.
4. In each `<tr>` row within any table: extract `<code>` text as display name, `<a href="*-full.bin">` as URL. A row is only accepted if it yields both a non-empty code text and a `-full.bin` href.
5. Resolve href to absolute URL: `base_url + "/" + href.lstrip("/")`.
6. Deduplicate display names by appending `(2)`, `(3)`, etc. for duplicates.

**Pattern matching** (`resolve_firmware_url` with a name other than `main`/`dev2`):
1. Try exact match on display name.
2. Try `fnmatch` pattern match on lowercased display name.
3. Try substring match.
4. Return single match; if none or multiple → error with full list.

**Download:** `urllib.request.urlopen` with 120s timeout. Reads entire response body into memory, writes to `tempfile.NamedTemporaryFile`. Caller is responsible for deleting the temp file.

### 5.5 LittleFS Image Handling

**Constants:**
- `BLOCK_SIZE = 4096` (matches ESP32 flash sector size)
- `PARTITION_SPIFFS_OFFSET = 0x320000`
- `PARTITION_SPIFFS_SIZE = 0xD0000`
- `block_count = 0xD0000 // 4096 = 208`

**Packing** (`pack_directory(src_dir, image_path, block_count)`):
1. Create `LittleFS(block_size=4096, block_count=208, disk_version=0x00020000)`.
   - `disk_version=0x00020000` forces LittleFS **v2.0** on-disk format (required by ESP32 Arduino LittleFS driver; default in newer library versions is v2.1).
2. `os.walk(src_dir, topdown=True)` to enumerate files and directories.
3. For each subdirectory: `fs.mkdir("/" + rel_path)`.
4. For each file: `fs.open(fs_path, "wb")` and write binary content.
5. Write `fs.context.buffer` (exactly `block_count * BLOCK_SIZE` bytes) to `image_path`.

**Unpacking** (`unpack_bytes(data, dest_dir, block_count)`):
1. Create `LittleFS(block_size=4096, block_count=len(data)//4096, mount=False)`.
2. Assign `fs.context.buffer = bytearray(data)`.
3. Call `fs.mount()`.
4. Recursively walk `fs.scandir("/")`. `entry.type == 2` indicates a directory.
5. Write each file to `dest_dir`, preserving directory structure.

**`unpack_image(image_path, dest_dir, block_count)`**: reads file, calls `unpack_bytes`.

**`extract_from_backup(backup_path, dest_dir, offset, size)`**: seeks to `offset`, reads `size` bytes, calls `unpack_bytes` with `block_count = size // BLOCK_SIZE`. No temp file needed.

**`list_fs_from_bin`** (littlefs_helper) and **`bin_fs_helpers.list_fs_from_bin_auto`**: The Wi-Fi tab uses the latter to auto-detect LittleFS vs SPIFFS in a .bin (full backup or raw partition); lists root entries `{"name", "size", "dir"}`. SPIFFS is supported via `spiffs_reader`; partition table can be read from the image via `partition_table.py`.

**`read_file_from_bin`** / **`read_file_from_bin_auto`**: Same slice; reads one file from root. Used for "Upload from .bin" on the Wi-Fi tab.

**Extract from backup** (USB tab): Uses `bin_fs_helpers.extract_from_bin_auto` to detect LittleFS or SPIFFS and extract files to the chosen directory.

### 5.6 Wi-Fi HTTP Protocol

**Host resolution:** `socket.gethostbyname(host)` handles both IP addresses and mDNS names (`.local`). The GUI caches the resolved IP separately so "Test connection" uses the resolved IP if available.

**Base URL:** `http://{resolved_ip}/` (always HTTP, no HTTPS).

**Authentication (challenge-response):**
1. GET `{base_url}login.html`.
2. Extract `<input name="preauth" value="...">` from the response HTML.
3. Compute `auth = SHA256("{user}:{preauth}:{password}")` as hex digest.
4. POST `{base_url}login.html` with form fields `user`, `preauth`, `auth`.
5. HTTP 401 response → invalid credentials. Other success → authenticated session.

**File get:** GET `{base_url}file/{name}`. Returns raw bytes. 401 → permission denied; 404 → not found.

**File put:** POST `{base_url}file` as multipart form with field `data` = `(filename, file_bytes)`.

**Status test:** GET `{base_url}status.json` (5s timeout). Response body is JSON returned as string.

**Session:** `requests.Session()` is used to persist cookies across requests within one operation.

### 5.7 SD Card Protocol

**Directory listing:** GET `{base_url}files.json` or `{base_url}files.json?dir={path}`. Response is a JSON array of objects: `[{"name": "foo.txt", "dir": false, "size": 1234}, ...]`. `"dir": true` indicates a subdirectory.

**File download:** GET `{base_url}sd/{path}`. Response is raw file bytes. Local path mirrors SD path under `outdir`.

**Recursive collection:** `collect_files_recursive` calls `list_dir` and recurses into subdirectories, building a flat list of all file paths. Used by both "Download all" in GUI and `sd fetch` in CLI.

**`resolve_sd_path_to_list`:** Lists the parent directory of the given path, checks whether the entry is a file or directory, and returns either `[path]` or the recursive expansion.

### 5.8 GUI Concurrency Model

**Background tasks:** `_run_in_background(work, on_done, on_progress=None, widget=None)` starts `work()` in a daemon thread. The thread's return value (a tuple) is passed to `on_done()` on the main thread via `widget.after(0, ...)` when `widget` is provided (or `tk._default_root` otherwise); the callback is skipped if the target no longer exists. If `work()` raises an exception, `on_done(False, str(exception))` is called. Call sites should pass `widget=self` so callbacks run on the main thread and are skipped when the frame is destroyed.

**Serial reader:** A dedicated daemon thread reads from `serial.Serial` in a loop. Data is put into a `queue.Queue`. `_drain_serial_queue()` is scheduled via `self.after(200, _drain_serial_queue)` and processes up to all available items in one call, appending to the `CTkTextbox`. This prevents GUI freezing from high-frequency serial data.

**Widget updates from threads:** All widget mutations (`configure`, `insert`, `delete`, etc.) happen on the main thread only, either via `self.after(0, lambda: ...)` closures or inside `on_done` callbacks.

**Serial log capture:** The `on_line` callback in esptool operations uses `self.after(0, lambda l=line: self._log(l))` with a default-argument capture of `line` to avoid late-binding closure bugs.

### 5.9 ANSI Escape Code Handling

The Flash/USB log area implements a two-phase ANSI processor.

**Phase 1 – Segment parsing** (`_parse_ansi_to_actions_debug`):
Scans the input string for ANSI escape sequences using a regex `\x1b\[([0-9;]*)([A-Za-z])`. Each character or escape produces a `(kind, text, tag)` tuple:
- `kind = "insert"`: text to insert, tag = ANSI color tag name or `""`.
- `kind = "el"`: Erase in Line signal (no text).
- `kind = "cuu"`: Cursor Up signal (n lines; treated the same as `el` for this use case).

**Phase 2 – Widget insertion** (`_log_insert_el_aware`):
For each action:
- `"insert"`: calls `widget.insert("end", text, (tag,) if tag else ())`.
- `"el"`: deletes from the start of the previous line up to (but not including) the trailing `\n`, then inserts the next text in its place. This overwrites the progress line in-place, achieving terminal-like behavior for esptool's `Writing at ...` progress updates.

**Erase-in-line detail:**
```
Before:  ...previous text\n   ← this line must be erased on [EL]
After:   ...                  ← line erased, cursor at position after \n of prior line
```
Implementation: `widget.delete("end-2l linestart", "end-1l linestart")` deletes exactly one line (including its newline) before the last line, then the subsequent insert writes the new progress text.

**Color tags** (`_configure_ansi_tags_for_log`):
Configures `CTkTextbox` tags: `ansi_red`, `ansi_green`, `ansi_yellow`, `ansi_blue`, `ansi_magenta`, `ansi_cyan`, `ansi_white`, `ansi_bright_red`, … (bright variants). Tags are applied via `widget.insert("end", text, (tag,))`.

**SGR code mapping:**

| Code | Tag |
|------|-----|
| 0 | reset (no tag) |
| 31 | ansi_red |
| 32 | ansi_green |
| 33 | ansi_yellow |
| 34 | ansi_blue |
| 35 | ansi_magenta |
| 36 | ansi_cyan |
| 37 | ansi_white |
| 91–97 | ansi_bright_red … ansi_bright_white |

---

## 6. Partition Layout

Based on `partitions-rdz.csv` (active firmware):

| Name | Type | SubType | Offset | Size | Notes |
|------|------|---------|--------|------|-------|
| nvs | data | nvs | 0x9000 | 0x5000 | Non-volatile storage |
| otadata | data | ota | 0xE000 | 0x2000 | OTA metadata |
| app0 | app | ota_0 | 0x10000 | 0x140000 | Primary application |
| app1 | app | ota_1 | 0x150000 | 0x140000 | OTA secondary slot |
| fonts | data | undefined | 0x310000 | 0x10000 | Font data |
| **spiffs** | data | spiffs | **0x320000** | **0xD0000** | **LittleFS filesystem** |
| coredump | data | coredump | 0x3F0000 | 0x10000 | Crash dumps |

> **Note:** An older partition table (`partitions-esp32v1.csv`) has spiffs at `0x290000` size `0x170000`. The code can read the partition table from a `.bin` image (`partition_table.py`) to locate the filesystem partition; SPIFFS and LittleFS are both supported for list/read/extract from backup or when viewing a .bin on the Wi-Fi tab.

The "code update" operation uses app0: offset `0x10000`, size `0x140000`.

The full backup reads `0x1000`–`0x3FF000` (4,190,208 bytes), which covers the bootloader and all partitions.

---

## 7. File Sets

Defined in `wifi_ops.FILE_SETS`:

| Set name | Files |
|----------|-------|
| `config` | `config.txt` |
| `qrg` | `qrg.txt` |
| `wifi` | `networks.txt` |
| `screens` | `screens1.txt`, `screens2.txt`, `screens3.txt`, `screens4.txt`, `screens5.txt` |
| `allconfig` | `config.txt`, `qrg.txt`, `networks.txt` |
| `all` | `config.txt`, `qrg.txt`, `networks.txt`, `screens1.txt`–`screens5.txt` |

---

## 8. Known Limitations and Future Work

| Area | Limitation / TODO |
|------|-------------------|
| Partition table | Partition table is read from `.bin` images (`partition_table.py`) to find filesystem offset/size. Reading from a live device is not implemented. |
| IMPROV | "Setup via IMPROV" button is on the **Serial** tab (left of "Save to file"); opens IMPROV dialog (device info, WiFi scan, connect). CLI `improv` subcommand is not implemented. |
| `extractfs` CLI | No `--keep-image` flag (not needed since no temp file is created). |
| Backup read baud | `read_backup` defaults to `921600` baud (via `DEFAULT_BAUD_READ`). Override with `--baud` on CLI or the esptool read baud rate setting in the GUI. "Use --no-stub" for read is checked by default. |
| Wi-Fi device list | `wifi_ops.list_files()` calls GET `files.json?dir=.int`; if empty or error, shows static list and status message "Listing not supported by device firmware; showing default file names." |
| `font` partition | `--part fonts` flash-type option is planned but not implemented. |
| Filesystem type | Partition is named `spiffs` in the table but contains a LittleFS v2.0 image. The two are incompatible. |
| `put --kind` CLI | Uploading a named file set via CLI is wired but individual file error reporting is basic. |
| SD card UI | Treeview with "..", column sort (Name/Size/Date), and paginated `/files.json`; directory navigation by double-click. |
| LittleFS version | `unpack_bytes` does not enforce v2.0 disk version on read; `littlefs-python` will mount v2.0 or v2.1. |
| Temp file cleanup | `downloadfs` always retains the raw image on failure for debugging. |
| Serial default baud | Default baud for the Serial tab is `115200` when no settings file exists; otherwise the saved `baud_rate` is used. Only the Serial tab combo controls it (no Settings-tab control); the value is saved on app close. |

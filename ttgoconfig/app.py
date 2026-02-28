"""
rdzTTGOsonde Desktop Flasher – UI implementation per plan.

Top-level: Flash | USB | Wi-Fi | Serial | Settings.
Settings persisted; Flash/USB use esptool; Serial uses pyserial; Wi-Fi uses HTTP.

Run: python -m ttgoconfig.app
"""

import os
import queue
import re
import threading
import datetime
import tkinter as tk
from tkinter import filedialog, messagebox, simpledialog, ttk
from typing import Any, Callable, Dict, List, Optional, Tuple

import customtkinter as ctk

from . import settings as settings_mod
from .esptool_helper import (
    flash_firmware, flash_app_partition, read_backup, write_backup,
    download_filesystem, upload_filesystem,
    PARTITION_SPIFFS_OFFSET, PARTITION_SPIFFS_SIZE,
    set_no_stub, set_no_stub_read, set_no_stub_write,
)
from .littlefs_helper import (
    pack_directory as lfs_pack_directory,
    unpack_image as lfs_unpack_image,
    extract_from_backup as lfs_extract_from_backup,
    list_fs_from_bin as lfs_list_fs_from_bin,
    BACKUP_FLASH_BASE as LFS_BACKUP_FLASH_BASE,
    BLOCK_SIZE as LFS_BLOCK_SIZE,
)
from .bin_fs_helpers import (
    list_fs_from_bin_auto,
    read_file_from_bin_auto,
    extract_from_bin_auto,
)
from .firmware import (
    fetch_manifest_firmware_url as _fetch_firmware_url_for_source,
    fetch_download_html_versions as _fetch_download_html_versions,
    download_firmware_to_temp,
)
from . import wifi_ops as wifi_ops_mod
from . import sd_ops as sd_ops_mod
from .bin_version_poc import get_version_from_bin
from . import improv_serial as improv_serial_mod

# Compact labels for the top bar
MODE_VALUES = ["Flash", "Serial", "USB", "Wi-Fi", "SD (via WiFi)", "Settings"]


def _strip_ansi_for_log(s: str) -> str:
    """Remove ANSI escape sequences so esptool output is readable in log areas."""
    s = re.sub(r"\x1b\[[0-9;]*[a-zA-Z]", "", s)
    s = re.sub(r"\x1b\[K", "", s)
    return s


# Actions for Flash/USB log: parse ANSI, [EL] triggers delete-previous-line, other escapes and <LF>/<CR> not shown
def _parse_ansi_to_actions_debug(s: str) -> list[tuple[str, str, str]]:
    """Parse ANSI and emit only ('insert', text, tag). Escapes become [EL], [SGR:31], etc. No erase handling."""
    sgr_names = {
        "0": "reset", "1": "bold",
        "31": "red", "32": "green", "33": "yellow", "34": "blue",
        "35": "magenta", "36": "cyan", "37": "white",
        "91": "red", "92": "green", "93": "yellow", "94": "blue",
        "95": "magenta", "96": "cyan", "97": "white",
    }
    actions: list[tuple[str, str, str]] = []
    text_buf = ""
    i = 0
    while i < len(s):
        if s[i:i + 2] == "\x1b[":
            if text_buf:
                actions.append(("insert", text_buf, ""))
                text_buf = ""
            j = i + 2
            while j < len(s) and s[j] not in "mABCDEFGHJK":
                j += 1
            if j < len(s):
                if s[j] == "K":
                    # \x1b[K or \x1b[2K
                    sub = s[i + 2:j]
                    actions.append(("insert", "[EL]" if sub.strip() in ("", "2") else "[EL:" + sub + "]", ""))
                elif s[j] == "m":
                    codes = s[i + 2:j].split(";")
                    parts = [sgr_names.get(c.strip(), c.strip()) for c in codes if c.strip()]
                    actions.append(("insert", "[SGR:" + ";".join(parts) + "]", ""))
                elif s[j] == "A":
                    n = s[i + 2:j].strip() or "1"
                    actions.append(("insert", "[CUU:" + n + "]", ""))  # cursor up
                elif s[j] == "B":
                    n = s[i + 2:j].strip() or "1"
                    actions.append(("insert", "[CUD:" + n + "]", ""))
                elif s[j] == "C":
                    n = s[i + 2:j].strip() or "1"
                    actions.append(("insert", "[CUF:" + n + "]", ""))
                elif s[j] == "D":
                    n = s[i + 2:j].strip() or "1"
                    actions.append(("insert", "[CUB:" + n + "]", ""))
                elif s[j] == "H":
                    actions.append(("insert", "[CUP]", ""))
                elif s[j] == "J":
                    actions.append(("insert", "[ED]", ""))
            i = j + 1
        else:
            text_buf += s[i]
            i += 1
    if text_buf:
        actions.append(("insert", text_buf, ""))
    return actions


def _is_el_tag(seg_text: str) -> bool:
    """True if this segment is the [EL] (erase line) tag — then we delete the preceding line before inserting."""
    return seg_text == "[EL]" or (seg_text.startswith("[EL:") and "]" in seg_text)


def _is_suppressed_tag(seg_text: str) -> bool:
    """True if this segment is a debug/escape tag we should not show in the log."""
    return (
        seg_text == "[CUP]" or seg_text == "[ED]"
        or seg_text.startswith("[EL]") or seg_text.startswith("[SGR:")
        or seg_text.startswith("[CUU:") or seg_text.startswith("[CUD:")
        or seg_text.startswith("[CUF:") or seg_text.startswith("[CUB:")
    )


def _log_insert_el_aware(
    widget: Any,
    tb: Any,
    seg_text: str,
    tag: str,
) -> None:
    """Insert text; if seg_text is [EL] or [EL:...], delete the preceding line first. Don't show debug tags or <LF>/<CR>."""
    if _is_el_tag(seg_text):
        try:
            try:
                start_idx = widget.index("end-2l linestart")
                end_idx = widget.index("end-1l linestart")
            except Exception:
                start_idx = widget.index("end-1l linestart")
                end_idx = widget.index("end")
            widget.delete(start_idx, end_idx)
        except Exception:
            pass
        return
    if _is_suppressed_tag(seg_text):
        return
    if tag:
        tb.insert("end", seg_text, tag)
    else:
        tb.insert("end", seg_text)


def _parse_ansi_to_actions(s: str) -> list[tuple[str, ...]]:
    """Parse ANSI: emit ('erase',) for \\x1b[K (Erase in Line), ('insert', text, tag) for colored text."""
    code_to_tag = {
        "31": "ansi_red", "32": "ansi_green", "33": "ansi_yellow", "34": "ansi_blue",
        "35": "ansi_magenta", "36": "ansi_cyan", "37": "ansi_white", "1": "ansi_bold",
        "91": "ansi_red", "92": "ansi_green", "93": "ansi_yellow", "94": "ansi_blue",
        "95": "ansi_magenta", "96": "ansi_cyan", "97": "ansi_white",
    }
    actions: list[tuple[str, ...]] = []
    text_buf = ""
    current_tag = ""
    i = 0
    while i < len(s):
        if s[i:i + 2] == "\x1b[":
            if text_buf:
                actions.append(("insert", text_buf, current_tag))
                text_buf = ""
            j = i + 2
            while j < len(s) and s[j] not in "mABCDEFGHJK":
                j += 1
            if j < len(s):
                if s[j] == "K":
                    # \x1b[K (erase to EOL) or \x1b[2K (erase entire line) - both mean "erase current line"
                    actions.append(("erase",))
                elif s[j] == "m":
                    codes = s[i + 2:j].split(";")
                    for c in codes:
                        c = c.strip()
                        if not c:
                            continue
                        if c == "0":
                            current_tag = ""
                            break
                        if c in code_to_tag:
                            current_tag = code_to_tag[c]
                            break
            i = j + 1
        else:
            text_buf += s[i]
            i += 1
    if text_buf:
        actions.append(("insert", text_buf, current_tag))
    return actions


def _configure_ansi_tags_for_log(widget: Any) -> None:
    """Configure ANSI color tags on a CTkTextbox (uses underlying _textbox)."""
    try:
        tb = getattr(widget, "_textbox", None) or widget
        tb.tag_configure("ansi_red", foreground="#e74c3c")
        tb.tag_configure("ansi_green", foreground="#27ae60")
        tb.tag_configure("ansi_yellow", foreground="#f1c40f")
        tb.tag_configure("ansi_blue", foreground="#3498db")
        tb.tag_configure("ansi_magenta", foreground="#9b59b6")
        tb.tag_configure("ansi_cyan", foreground="#1abc9c")
        tb.tag_configure("ansi_white", foreground="#ecf0f1")
        tb.tag_configure("ansi_bold", foreground="#2c3e50")
    except Exception:
        pass


def _parse_ansi_to_segments(s: str) -> list[tuple[str, str]]:
    """Parse string with ANSI SGR codes; return list of (text, tag_name). tag_name '' = default. For Serial log."""
    out: list[tuple[str, str]] = []
    code_to_tag = {
        "31": "ansi_red", "32": "ansi_green", "33": "ansi_yellow", "34": "ansi_blue",
        "35": "ansi_magenta", "36": "ansi_cyan", "37": "ansi_white", "1": "ansi_bold",
        "91": "ansi_red", "92": "ansi_green", "93": "ansi_yellow", "94": "ansi_blue",
        "95": "ansi_magenta", "96": "ansi_cyan", "97": "ansi_white",
    }
    text_buf = ""
    current_tag = ""
    i = 0
    while i < len(s):
        if s[i:i + 2] == "\x1b[":
            if text_buf:
                out.append((text_buf, current_tag))
                text_buf = ""
            j = i + 2
            while j < len(s) and s[j] not in "mABCDEFGHJK":
                j += 1
            if j < len(s) and s[j] == "m":
                codes = s[i + 2:j].split(";")
                for c in codes:
                    c = c.strip()
                    if not c:
                        continue
                    if c == "0":
                        current_tag = ""
                    elif c in code_to_tag:
                        current_tag = code_to_tag[c]
            i = j + 1
        else:
            text_buf += s[i]
            i += 1
    if text_buf:
        out.append((text_buf, current_tag))
    return out


def get_serial_ports() -> list[str]:
    """Return list of serial port names for dropdowns. Placeholder: fake list if pyserial missing."""
    try:
        import serial.tools.list_ports as list_ports
        ports = [p.device for p in list_ports.comports()]
        return ["Auto"] + ports if ports else ["Auto", "(no ports found)"]
    except Exception:
        return ["Auto", "COM1", "/dev/cu.usbserial-0001", "(mock)"]


class ContentFrame(ctk.CTkFrame):
    """Base for mode content; one per mode."""

    def __init__(self, master, mode_name: str, app: Optional["App"] = None, **kwargs):
        super().__init__(master, fg_color="transparent", **kwargs)
        self.mode_name = mode_name
        self.app = app  # type: Optional[App]
        self._log_widget: Optional[Any] = None

    def show(self) -> None:
        self.grid(row=0, column=0, sticky="nsew")

    def hide(self) -> None:
        self.grid_remove()

    def _make_log_widget(self, **kwargs) -> ctk.CTkTextbox:
        """Create a CTkTextbox, configure ANSI color tags, and register as self._log_widget."""
        widget = ctk.CTkTextbox(self, **kwargs)
        _configure_ansi_tags_for_log(widget)
        self._log_widget = widget
        return widget

    def _log(self, text: str) -> None:
        widget = self._log_widget
        if widget is None:
            return
        actions = _parse_ansi_to_actions_debug(text)
        tb = getattr(widget, "_textbox", None) or widget
        widget.configure(state="normal")
        for _act in actions:
            _, seg_text, tag = _act
            _log_insert_el_aware(widget, tb, seg_text, tag)
        widget.see("end")
        widget.configure(state="disabled")

    def _refresh_ports(self) -> None:
        if not hasattr(self, "port_combo"):
            return
        current = self.port_combo.get()
        ports = get_serial_ports()
        self.port_combo.configure(values=ports)
        if self.app and hasattr(self.app, "port_var"):
            saved = (self.app.settings.get("last_port") or "").strip()
            if saved and saved in ports:
                self.app.port_var.set(saved)
            elif current in ports:
                self.app.port_var.set(current)
            elif ports:
                self.app.port_var.set(ports[0])
        elif current in ports:
            self.port_combo.set(current)

    def _on_line_cb(self) -> Callable[[str], None]:
        """Return a thread-safe on_line callback that appends to this frame's log."""
        return lambda line: self.after(0, lambda l=line: self._log(l))

    def _make_port_combo_row(self, parent: Any) -> int:
        """Add Port label, port_combo, and Refresh button to parent at row 0. Returns next available column index."""
        parent.grid_columnconfigure(1, weight=1)
        ctk.CTkLabel(parent, text="Port:").grid(row=0, column=0, padx=(0, 10), pady=5)
        self.port_combo = ctk.CTkComboBox(
            parent, values=get_serial_ports(), width=220,
            variable=self.app.port_var if self.app else None
        )
        self.port_combo.grid(row=0, column=1, padx=0, pady=5, sticky="w")
        ctk.CTkButton(parent, text="Refresh", width=80, command=self._refresh_ports).grid(
            row=0, column=2, padx=(10, 0), pady=5
        )
        return 3

    def _initial_dir(self) -> str:
        """Default directory for save/extract dialogs: last used, or home."""
        if not self.app:
            return os.path.expanduser("~")
        raw = (self.app.settings.get("last_directory") or "").strip()
        if raw and os.path.isdir(raw):
            return raw
        return os.path.expanduser("~")

    def _set_last_directory(self, path: str) -> None:
        """Remember chosen path for next dialog (directory or file's parent)."""
        if not path or not self.app:
            return
        dir_path = path if os.path.isdir(path) else os.path.dirname(path)
        if dir_path and os.path.isdir(dir_path):
            self.app.settings["last_directory"] = dir_path
            settings_mod.save(self.app.settings)

    def _backup_initial_dir(self) -> str:
        """Initial directory for backup-related dialogs: configured backup folder."""
        if not self.app:
            return os.path.expanduser("~")
        folder = settings_mod.get_backup_folder_expanded(self.app.settings)
        return folder if folder else os.path.expanduser("~")


def _run_in_background(
    work: Callable[..., tuple[bool, str]],
    on_done: Callable[[bool, str], None],
    on_progress: Optional[Callable[[int, int], None]] = None,
    widget: Optional[tk.Misc] = None,
) -> None:
    """Run work() in a thread; on_done(ok, msg) on main thread. If on_progress is set, work(progress_cb) receives a callback progress_cb(n, total). Uses widget for after() when provided so callbacks run on main thread and are skipped if widget is destroyed."""
    def run():
        target = widget if widget is not None else tk._default_root
        try:
            if on_progress is not None:
                def progress_cb(n: int, total: int) -> None:
                    if target:
                        target.after(0, lambda n=n, total=total: _safe_callback(target, on_progress, (n, total)))
                ok, msg = work(progress_cb)
            else:
                ok, msg = work()
        except Exception as e:
            ok, msg = False, str(e)
        if on_done and target:
            target.after(0, lambda: _safe_callback(target, on_done, (ok, msg)))
    def _safe_callback(target: tk.Misc, cb: Callable, args: tuple) -> None:
        if target and target.winfo_exists() and cb:
            cb(args[0], args[1])
    t = threading.Thread(target=run, daemon=True)
    t.start()


class FlashFrame(ContentFrame):
    def __init__(self, master, app: Optional["App"] = None, **kwargs):
        super().__init__(master, "Flash", app=app, **kwargs)
        self.grid_columnconfigure(0, weight=1)

        row = 0
        dev_frame = ctk.CTkFrame(self, fg_color="transparent")
        dev_frame.grid(row=row, column=0, sticky="ew", padx=20, pady=(20, 10))
        self._make_port_combo_row(dev_frame)
        row += 1

        src_label = ctk.CTkLabel(self, text="Firmware source:")
        src_label.grid(row=row, column=0, sticky="w", padx=20, pady=(10, 5))
        row += 1
        self.fw_source = ctk.CTkSegmentedButton(
            self, values=["Download from website", "Local file", "From backup"],
            command=self._on_fw_source_change, dynamic_resizing=False,
        )
        self.fw_source.set("Download from website")
        self.fw_source.grid(row=row, column=0, sticky="ew", padx=20, pady=5)
        row += 1

        self.fw_opts_frame = ctk.CTkFrame(self, fg_color="transparent")
        self.fw_opts_frame.grid(row=row, column=0, sticky="ew", padx=20, pady=10)
        self.fw_opts_frame.grid_columnconfigure(0, weight=1)
        self._build_fw_opts()
        row += 1

        flash_type_frame = ctk.CTkFrame(self, fg_color="transparent")
        flash_type_frame.grid(row=row, column=0, sticky="ew", padx=20, pady=(5, 10))
        flash_type_frame.grid_columnconfigure(1, weight=1)
        ctk.CTkLabel(flash_type_frame, text="What to flash:").grid(row=0, column=0, padx=(0, 10), pady=5)
        self.flash_type = ctk.CTkComboBox(
            flash_type_frame, values=["Full image", "Code update"], width=180
        )
        self.flash_type.set("Full image")
        self.flash_type.grid(row=0, column=1, sticky="w", pady=5)
        row += 1

        self.flash_btn = ctk.CTkButton(
            self, text="Flash", height=40, font=ctk.CTkFont(size=14, weight="bold"), command=self._do_flash
        )
        self.flash_btn.grid(row=row, column=0, padx=20, pady=20, sticky="ew")
        row += 1

        self.log_text = self._make_log_widget(height=120, state="disabled")
        self.log_text.grid(row=row, column=0, sticky="nsew", padx=20, pady=(0, 20))
        self.grid_rowconfigure(row, weight=1)
        self._flash_running = False

    def _build_fw_opts(self) -> None:
        for w in self.fw_opts_frame.winfo_children():
            w.destroy()
        src = self.fw_source.get()
        if src == "Download from website":
            self._download_fetched_versions = {}  # display -> url
            self.download_combo = ctk.CTkComboBox(
                self.fw_opts_frame, values=["Stable (main)", "Development (dev2)"], width=220
            )
            self.download_combo.grid(row=0, column=0, sticky="w")
            self.fetch_older_btn = ctk.CTkButton(
                self.fw_opts_frame, text="Fetch list of older versions",
                width=260, command=self._fetch_older_versions
            )
            self.fetch_older_btn.grid(row=0, column=1, sticky="w", padx=(10, 0))
        elif src == "Local file":
            self.local_path_var = ctk.StringVar(value="")
            ctk.CTkEntry(self.fw_opts_frame, textvariable=self.local_path_var, width=400, state="readonly").grid(
                row=0, column=0, sticky="ew", padx=(0, 10)
            )
            ctk.CTkButton(self.fw_opts_frame, text="Browse…", width=80, command=self._browse_local).grid(
                row=0, column=1, padx=0
            )
            self.fw_opts_frame.grid_columnconfigure(0, weight=1)
        else:  # From backup
            self._refresh_backup_list()

    def _refresh_backup_list(self) -> None:
        for w in self.fw_opts_frame.winfo_children():
            w.destroy()
        s = self.app.settings if self.app else {}
        folder = settings_mod.get_backup_folder_expanded(s)
        backups = []
        if os.path.isdir(folder):
            try:
                backups = sorted(
                    [f for f in os.listdir(folder) if f.endswith(".bin")],
                    key=lambda f: os.path.getmtime(os.path.join(folder, f)),
                    reverse=True,
                )
            except OSError:
                pass
        if not backups:
            ctk.CTkLabel(self.fw_opts_frame, text="No backups in backup folder.", text_color="gray").grid(
                row=0, column=0, sticky="w"
            )
            self.backup_combo = None
        else:
            display_values = []
            for f in backups:
                full_path = os.path.join(folder, f)
                version = get_version_from_bin(full_path)
                display_values.append("%s (%s)" % (f, version) if version else f)
            self.backup_combo = ctk.CTkComboBox(self.fw_opts_frame, values=display_values, width=280)
            self.backup_combo.grid(row=0, column=0, sticky="w")
        self.fw_opts_frame.grid_columnconfigure(0, weight=1)

    def _on_fw_source_change(self, _value: str) -> None:
        self._build_fw_opts()

    def _fetch_older_versions(self) -> None:
        if not hasattr(self, "fetch_older_btn") or not hasattr(self, "download_combo"):
            return
        s = self.app.settings if self.app else {}
        base_url = s.get("download_url", "").strip() or settings_mod.DEFAULT_DOWNLOAD_URL
        self.fetch_older_btn.configure(state="disabled")

        def work() -> tuple[bool, str, list[tuple[str, str]]]:
            rows, err = _fetch_download_html_versions(base_url)
            if err:
                return False, err, []
            return True, "", rows

        def on_done(ok: bool, msg: str, rows: list[tuple[str, str]]) -> None:
            self.fetch_older_btn.configure(state="normal")
            if not ok:
                messagebox.showerror("Fetch versions", msg)
                return
            self._download_fetched_versions = {display: url for display, url in rows}
            default = ["Stable (main)", "Development (dev2)"]
            displays = default + [display for display, _ in rows]
            self.download_combo.configure(values=displays)

        def run() -> None:
            try:
                ok, msg, rows = work()
            except Exception as e:
                ok, msg, rows = False, str(e), []
            if self.winfo_exists():
                self.after(0, lambda: on_done(ok, msg, rows))
        threading.Thread(target=run, daemon=True).start()

    def _browse_local(self) -> None:
        path = filedialog.askopenfilename(
            title="Select firmware image – choose .bin file",
            initialdir=self._initial_dir(),
            filetypes=[("Binary", "*.bin"), ("All", "*.*")],
        )
        if path:
            self._set_last_directory(path)
            self.local_path_var.set(path)

    def _get_image_path(self) -> Optional[str]:
        src = self.fw_source.get()
        if src == "Download from website":
            return None  # handled in _do_flash via download to temp
        if src == "Local file":
            p = self.local_path_var.get().strip() if hasattr(self, "local_path_var") else ""
            if not p or not os.path.isfile(p):
                messagebox.showwarning("Flash", "Please select a local .bin file.")
                return None
            return p
        if src == "From backup":
            if not hasattr(self, "backup_combo") or self.backup_combo is None:
                messagebox.showwarning("Flash", "No backups found. Use Manage (USB) → Make backup first.")
                return None
            name = self.backup_combo.get()
            if " (" in name:
                name = name.split(" (")[0].strip()
            folder = settings_mod.get_backup_folder_expanded(self.app.settings if self.app else {})
            return os.path.join(folder, name)
        return None

    def _do_flash(self) -> None:
        src = self.fw_source.get()
        port = self.port_combo.get()
        s = self.app.settings if self.app else {}
        baud = s.get("baud_rate_write", settings_mod.DEFAULT_BAUD_WRITE)

        if src == "Download from website":
            choice = self.download_combo.get() if hasattr(self, "download_combo") else "Stable (main)"
            base_url = s.get("download_url", "").strip() or settings_mod.DEFAULT_DOWNLOAD_URL
            self._flash_running = True
            self.flash_btn.configure(state="disabled")
            self._log("\n--- Download from website (%s) ---\n" % choice)

            def work():
                fetched = getattr(self, "_download_fetched_versions", None) or {}
                if choice in fetched:
                    url = fetched[choice]
                else:
                    url, err = _fetch_firmware_url_for_source(base_url, choice)
                    if err or not url:
                        return False, err or "No firmware URL"
                self.after(0, lambda: self._log("Downloading %s...\n" % url))
                temp_path, err = download_firmware_to_temp(url)
                if err:
                    return False, err
                try:
                    code_only = self.flash_type.get() == "Code update"
                    if code_only:
                        ok, msg = flash_app_partition(
                            port, baud, temp_path,
                            on_line=self._on_line_cb(),
                        )
                    else:
                        ok, msg = flash_firmware(
                            port, baud, temp_path,
                            on_line=self._on_line_cb(),
                        )
                    return ok, msg
                finally:
                    try:
                        os.unlink(temp_path)
                    except Exception:
                        pass

            def on_done(ok: bool, msg: str) -> None:
                self._flash_running = False
                self.flash_btn.configure(state="normal")
                if ok:
                    self._log("Flashed successfully.\n")
                    messagebox.showinfo("Flash", "Flashed successfully.")
                else:
                    self._log("Error: %s\n" % msg)
                    messagebox.showerror("Flash", msg or "Flash failed. Check the log.")

            _run_in_background(work, on_done, widget=self)
            return

        path = self._get_image_path()
        if not path:
            return
        self._flash_running = True
        self.flash_btn.configure(state="disabled")
        self._log("\n--- Flashing %s ---\n" % path)

        def work():
            code_only = self.flash_type.get() == "Code update"
            if code_only:
                return flash_app_partition(port, baud, path, on_line=self._on_line_cb())
            return flash_firmware(port, baud, path, on_line=self._on_line_cb())

        def on_done(ok: bool, _msg: str) -> None:
            self._flash_running = False
            self.flash_btn.configure(state="normal")
            if ok:
                self._log("Flashed successfully.\n")
                messagebox.showinfo("Flash", "Flashed successfully.")
            else:
                messagebox.showerror("Flash", "Flash failed. Check the log.")

        _run_in_background(work, on_done, widget=self)


class USBManageFrame(ContentFrame):
    def __init__(self, master, app: Optional["App"] = None, **kwargs):
        super().__init__(master, "USB", app=app, **kwargs)
        self.grid_columnconfigure(0, weight=1)

        row = 0
        dev_frame = ctk.CTkFrame(self, fg_color="transparent")
        dev_frame.grid(row=row, column=0, sticky="ew", padx=20, pady=(20, 10))
        self._make_port_combo_row(dev_frame)
        row += 1

        actions_frame = ctk.CTkFrame(self, fg_color="transparent")
        actions_frame.grid(row=row, column=0, sticky="ew", padx=20, pady=(15, 5))
        actions_frame.grid_columnconfigure(0, weight=1, uniform="col")
        actions_frame.grid_columnconfigure(1, weight=1, uniform="col")

        ctk.CTkLabel(actions_frame, text="TTGO→PC", font=ctk.CTkFont(weight="bold")).grid(
            row=0, column=0, sticky="w", padx=(0, 10), pady=(0, 6))
        ctk.CTkLabel(actions_frame, text="PC→TTGO", font=ctk.CTkFont(weight="bold")).grid(
            row=0, column=1, sticky="w", padx=(10, 0), pady=(0, 6))

        ctk.CTkButton(actions_frame, text="Make full backup", command=self._make_backup).grid(
            row=1, column=0, sticky="ew", padx=(0, 10), pady=3)
        ctk.CTkButton(actions_frame, text="Restore full backup", command=self._restore_full_backup).grid(
            row=1, column=1, sticky="ew", padx=(10, 0), pady=3)

        ctk.CTkButton(actions_frame, text="Extract filesystem", command=self._downloadfs).grid(
            row=2, column=0, sticky="ew", padx=(0, 10), pady=3)
        ctk.CTkButton(actions_frame, text="Upload filesystem", command=self._uploadfs).grid(
            row=2, column=1, sticky="ew", padx=(10, 0), pady=3)

        ctk.CTkFrame(actions_frame, height=1, fg_color=("gray70", "gray40")).grid(
            row=3, column=0, columnspan=2, sticky="ew", padx=0, pady=(10, 6))

        ctk.CTkButton(actions_frame, text="Extract filesystem from backup image",
                      command=self._extractfs_from_backup).grid(
            row=4, column=0, sticky="ew", padx=(0, 10), pady=3)

        row += 1
        self._op_log = self._make_log_widget(height=100, state="disabled")
        self._op_log.grid(row=row, column=0, sticky="nsew", padx=20, pady=(10, 20))
        self.grid_rowconfigure(row, weight=1)

    def _make_backup(self) -> None:
        folder = self._backup_initial_dir()
        os.makedirs(folder, exist_ok=True)
        from datetime import datetime
        default_name = "backup-%s.bin" % datetime.now().strftime("%Y%m%d-%H%M")
        path = filedialog.asksaveasfilename(
            title="Save backup as – choose location",
            initialdir=folder,
            initialfile=default_name,
            defaultextension=".bin",
            filetypes=[("Binary", "*.bin"), ("All", "*.*")],
        )
        if not path:
            return
        port = self.port_combo.get()
        baud = (self.app.settings if self.app else {}).get("baud_rate_read", settings_mod.DEFAULT_BAUD_READ)
        self._log("\n--- Make backup to %s ---\n" % path)

        def work():
            return read_backup(port, baud, path, on_line=self._on_line_cb())

        def on_done(ok: bool, _msg: str) -> None:
            if ok:
                self._log("Backup saved.\n")
                messagebox.showinfo("Backup", "Backup saved.")
            else:
                messagebox.showerror("Backup", "Backup failed. Check the log.")

        _run_in_background(work, on_done, widget=self)

    def _restore_full_backup(self) -> None:
        path = filedialog.askopenfilename(
            title="Restore full backup – select backup image (.bin)",
            initialdir=self._backup_initial_dir(),
            filetypes=[("Binary", "*.bin"), ("All", "*.*")],
        )
        if not path:
            return
        if not messagebox.askyesno("Restore", "Restore full backup from %s? This will overwrite the device flash." % path):
            return
        port = self.port_combo.get()
        baud = (self.app.settings if self.app else {}).get("baud_rate_write", settings_mod.DEFAULT_BAUD_WRITE)
        self._log("\n--- Restore from %s ---\n" % path)

        def work():
            return write_backup(port, baud, path, on_line=self._on_line_cb())

        def on_done(ok: bool, _msg: str) -> None:
            if ok:
                self._log("Restore complete.\n")
                messagebox.showinfo("Restore", "Restore complete.")
            else:
                messagebox.showerror("Restore", "Restore failed. Check the log.")

        _run_in_background(work, on_done, widget=self)

    def _restore_selected_files(self) -> None:
        messagebox.showinfo("Restore selected files", "Not yet implemented.")

    def _downloadfs(self) -> None:
        dest_dir = filedialog.askdirectory(
            title="Extract device filesystem – choose destination folder",
            initialdir=self._initial_dir(),
        )
        if not dest_dir:
            return
        port = self.port_combo.get()
        baud = (self.app.settings if self.app else {}).get("baud_rate_read", settings_mod.DEFAULT_BAUD_READ)
        block_count = PARTITION_SPIFFS_SIZE // LFS_BLOCK_SIZE
        self._log("\n--- Extract filesystem from device to %s ---\n" % dest_dir)
        import tempfile

        def work():
            tmp_fd, tmp_path = tempfile.mkstemp(suffix=".bin", prefix="lfs_dl_")
            os.close(tmp_fd)
            try:
                ok, msg = download_filesystem(
                    port, baud, tmp_path,
                    on_line=self._on_line_cb(),
                )
                if not ok:
                    return False, msg
                lfs_unpack_image(tmp_path, dest_dir, block_count)
                return True, ""
            finally:
                try:
                    os.unlink(tmp_path)
                except Exception:
                    pass

        def on_done(ok: bool, _msg: str) -> None:
            if ok:
                self._set_last_directory(dest_dir)
                self._log("Filesystem extracted to %s\n" % dest_dir)
                messagebox.showinfo("Extract filesystem", "Filesystem extracted to:\n%s" % dest_dir)
            else:
                messagebox.showerror("Extract filesystem", "Failed. Check the log.")

        _run_in_background(work, on_done, widget=self)

    def _uploadfs(self) -> None:
        src_dir = filedialog.askdirectory(
            title="Upload filesystem to device – choose source folder",
            initialdir=self._initial_dir(),
        )
        if not src_dir:
            return
        if not messagebox.askyesno(
            "Upload filesystem",
            "Upload '%s' as filesystem to device?\nThis will overwrite the current filesystem." % src_dir,
        ):
            return
        port = self.port_combo.get()
        baud = (self.app.settings if self.app else {}).get("baud_rate_write", settings_mod.DEFAULT_BAUD_WRITE)
        block_count = PARTITION_SPIFFS_SIZE // LFS_BLOCK_SIZE
        self._log("\n--- Upload filesystem from %s ---\n" % src_dir)
        import tempfile

        def work():
            tmp_fd, tmp_path = tempfile.mkstemp(suffix=".bin", prefix="lfs_ul_")
            os.close(tmp_fd)
            try:
                lfs_pack_directory(src_dir, tmp_path, block_count)
                self.after(0, lambda: self._log("LittleFS image built, flashing…\n"))
                ok, msg = upload_filesystem(
                    port, baud, tmp_path,
                    on_line=self._on_line_cb(),
                )
                return ok, msg
            finally:
                try:
                    os.unlink(tmp_path)
                except Exception:
                    pass

        def on_done(ok: bool, _msg: str) -> None:
            if ok:
                self._set_last_directory(src_dir)
                self._log("Filesystem uploaded successfully.\n")
                messagebox.showinfo("Upload filesystem", "Filesystem uploaded successfully.")
            else:
                messagebox.showerror("Upload filesystem", "Failed. Check the log.")

        _run_in_background(work, on_done, widget=self)

    def _extractfs_from_backup(self) -> None:
        backup_path = filedialog.askopenfilename(
            title="Extract from backup – select backup image (.bin)",
            initialdir=self._backup_initial_dir(),
            filetypes=[("Binary", "*.bin"), ("All", "*.*")],
        )
        if not backup_path:
            return
        dest_dir = filedialog.askdirectory(
            title="Extract from backup – choose destination folder",
            initialdir=self._backup_initial_dir(),
        )
        if not dest_dir:
            return
        self._log("\n--- Extract filesystem from backup %s to %s ---\n" % (backup_path, dest_dir))

        def work():
            ok, err = extract_from_bin_auto(backup_path, dest_dir)
            return ok, err or ""

        def on_done(ok: bool, msg: str) -> None:
            if ok:
                self._log("Filesystem extracted to %s\n" % dest_dir)
                messagebox.showinfo("Extract filesystem", "Filesystem extracted to:\n%s" % dest_dir)
            else:
                self._log("Error: %s\n" % msg)
                messagebox.showerror("Extract filesystem", "Failed: %s" % msg)

        _run_in_background(work, on_done, widget=self)


def _make_host_auth_row(
    parent: Any,
    app: "App",
    start_row: int,
    on_resolve: Callable[[], None],
    on_test: Callable[[], None],
    columnspan: int = 2,
) -> Tuple[int, Any, Any, Any, Any]:
    """Build shared Host + Auth (user/password/save) row. Returns (next_row, host_entry, ip_label, auth_user_entry, auth_pass_entry)."""
    row = start_row
    host_frame = ctk.CTkFrame(parent, fg_color="transparent")
    host_frame.grid(row=row, column=0, columnspan=columnspan, sticky="ew", padx=20, pady=(10, 4))
    host_frame.grid_columnconfigure(1, weight=1)
    ctk.CTkLabel(host_frame, text="Host:").grid(row=0, column=0, sticky="w", padx=(0, 8), pady=0)
    host_entry = ctk.CTkEntry(
        host_frame,
        textvariable=app.wifi_host_var,
        placeholder_text="rdzsonde.local or IP",
    )
    host_entry.grid(row=0, column=1, sticky="ew", padx=(0, 10), pady=0)
    ip_label = ctk.CTkLabel(host_frame, text="", text_color="gray", width=160, anchor="w")
    ip_label.grid(row=0, column=2, sticky="w", padx=(0, 10), pady=0)
    ctk.CTkButton(host_frame, text="Resolve", width=80, command=on_resolve).grid(row=0, column=3, padx=0, pady=0)
    ctk.CTkButton(host_frame, text="Test connection", width=120, command=on_test).grid(row=0, column=4, padx=(10, 0), pady=0)
    row += 1
    auth_frame = ctk.CTkFrame(parent, fg_color="transparent")
    auth_frame.grid(row=row, column=0, columnspan=columnspan, sticky="ew", padx=20, pady=(4, 4))
    auth_frame.grid_columnconfigure(1, weight=1)
    ctk.CTkLabel(auth_frame, text="Auth (optional):").grid(row=0, column=0, sticky="w", padx=(0, 8), pady=0)
    ctk.CTkLabel(auth_frame, text="User").grid(row=0, column=2, padx=(0, 4), pady=0)
    auth_user_entry = ctk.CTkEntry(
        auth_frame,
        width=120,
        textvariable=app.wifi_user_var,
        placeholder_text="username",
    )
    auth_user_entry.grid(row=0, column=3, padx=(0, 12), pady=0, sticky="w")
    ctk.CTkLabel(auth_frame, text="Password").grid(row=0, column=4, padx=(0, 4), pady=0)
    auth_pass_entry = ctk.CTkEntry(
        auth_frame,
        width=120,
        textvariable=app.wifi_pass_var,
        placeholder_text="password",
        show="•",
    )
    auth_pass_entry.grid(row=0, column=5, padx=0, pady=0, sticky="w")
    ctk.CTkCheckBox(
        auth_frame,
        text="Save password",
        variable=app.wifi_save_pass_var,
        width=0,
    ).grid(row=0, column=6, padx=(12, 0), pady=0, sticky="w")
    row += 1
    return row, host_entry, ip_label, auth_user_entry, auth_pass_entry


class FilesWiFiFrame(ContentFrame):
    """Wi-Fi tab: dual-pane file manager (local folder/bin | device files)."""

    def __init__(self, master, app: Optional["App"] = None, **kwargs):
        super().__init__(master, "Wi-Fi", app=app, **kwargs)
        self.grid_columnconfigure(0, weight=1)

        row, self.host_entry, self.ip_label, self.auth_user_entry, self.auth_pass_entry = _make_host_auth_row(
            self, self.app, 0, self._resolve_host, self._test_connection, columnspan=2
        )

        # Dual-pane: local folder (left) | device files (right)
        self.grid_columnconfigure(0, weight=1)
        self.grid_columnconfigure(1, weight=1)
        panes_row = row  # from _make_host_auth_row return
        self.grid_rowconfigure(panes_row, weight=1)

        # Left: local folder (header row + path line + tree aligned with right)
        left_frame = ctk.CTkFrame(self, fg_color="transparent")
        left_frame.grid(row=panes_row, column=0, sticky="nsew", padx=(20, 10), pady=8)
        left_frame.grid_columnconfigure(0, weight=1)
        left_frame.grid_rowconfigure(1, weight=1)
        left_head = ctk.CTkFrame(left_frame, fg_color="transparent")
        left_head.grid(row=0, column=0, sticky="ew", pady=(0, 4))
        left_head.grid_columnconfigure(0, weight=1)
        self._wifi_local_path = self._initial_dir()
        self._wifi_local_path_var = tk.StringVar(value=self._wifi_local_path)
        self._wifi_left_mode = "folder"  # "folder" | "bin"
        self._wifi_local_bin_path: Optional[str] = None
        self._wifi_local_path_label = ctk.CTkLabel(
            left_head, text="", anchor="w", font=ctk.CTkFont(weight="bold"),
        )
        self._wifi_local_path_label.grid(row=0, column=0, sticky="w")
        self._wifi_local_path_tooltip = None
        self._wifi_local_path_tooltip_after = None
        self._wifi_local_path_label.bind("<Enter>", self._wifi_local_path_show_tooltip)
        self._wifi_local_path_label.bind("<Leave>", self._wifi_local_path_hide_tooltip)
        local_container = tk.Frame(left_frame)
        local_container.grid(row=1, column=0, sticky="nsew", pady=(0, 4))
        local_container.grid_columnconfigure(0, weight=1)
        local_container.grid_rowconfigure(0, weight=1)
        self.wifi_local_tree = ttk.Treeview(local_container, columns=("size",), show="tree headings", selectmode="extended")
        self.wifi_local_tree.heading("#0", text="Name")
        self.wifi_local_tree.column("#0", width=180)
        self.wifi_local_tree.heading("size", text="Size")
        self.wifi_local_tree.column("size", width=70, anchor="e")
        local_yscroll = ttk.Scrollbar(local_container, orient="vertical", command=self.wifi_local_tree.yview)
        self.wifi_local_tree.configure(yscrollcommand=local_yscroll.set)
        self.wifi_local_tree.pack(side="left", fill="both", expand=True)
        local_yscroll.pack(side="right", fill="y")
        self.wifi_local_tree.bind("<Double-1>", self._wifi_local_on_double_click)

        # Right: device files (header row, tree aligned with left)
        right_frame = ctk.CTkFrame(self, fg_color="transparent")
        right_frame.grid(row=panes_row, column=1, sticky="nsew", padx=(10, 20), pady=8)
        right_frame.grid_columnconfigure(0, weight=1)
        right_frame.grid_rowconfigure(1, weight=1)
        right_head = ctk.CTkFrame(right_frame, fg_color="transparent")
        right_head.grid(row=0, column=0, sticky="ew", pady=(0, 4))
        right_head.grid_columnconfigure(0, weight=1)
        ctk.CTkLabel(right_head, text="Device files", font=ctk.CTkFont(weight="bold")).grid(row=0, column=0, sticky="w")
        device_container = tk.Frame(right_frame)
        device_container.grid(row=1, column=0, sticky="nsew", pady=(0, 4))
        device_container.grid_columnconfigure(0, weight=1)
        device_container.grid_rowconfigure(0, weight=1)
        self.wifi_device_tree = ttk.Treeview(device_container, columns=("size",), show="tree headings", selectmode="extended")
        self.wifi_device_tree.heading("#0", text="Name")
        self.wifi_device_tree.column("#0", width=180)
        self.wifi_device_tree.heading("size", text="Size")
        self.wifi_device_tree.column("size", width=70, anchor="e")
        device_yscroll = ttk.Scrollbar(device_container, orient="vertical", command=self.wifi_device_tree.yview)
        self.wifi_device_tree.configure(yscrollcommand=device_yscroll.set)
        self.wifi_device_tree.pack(side="left", fill="both", expand=True)
        device_yscroll.pack(side="right", fill="y")
        self._wifi_device_entries: List[dict] = []

        # Buttons row below tables, then status (columns equal width so buttons spread evenly)
        row = panes_row + 1
        btn_frame = ctk.CTkFrame(self, fg_color="transparent")
        btn_frame.grid(row=row, column=0, columnspan=2, sticky="ew", padx=20, pady=(8, 4))
        for c in range(5):
            btn_frame.grid_columnconfigure(c, weight=1)
        btn_frame.grid_rowconfigure(1, weight=0)
        ctk.CTkButton(btn_frame, text="Change", width=70, command=self._wifi_local_change).grid(row=0, column=0, padx=4, pady=0)
        ctk.CTkButton(btn_frame, text="Open .bin", width=80, command=self._wifi_local_open_bin).grid(row=0, column=1, padx=4, pady=0)
        self._wifi_upload_btn = ctk.CTkButton(
            btn_frame, text="Upload to device →", width=160, command=self._wifi_upload_selected
        )
        self._wifi_upload_btn.grid(row=0, column=2, padx=4, pady=0)
        self._wifi_download_btn = ctk.CTkButton(
            btn_frame, text="← Download", width=120, command=self._wifi_download_selected
        )
        self._wifi_download_btn.grid(row=0, column=3, padx=4, pady=0)
        ctk.CTkButton(btn_frame, text="Refresh", width=80, command=self._wifi_device_refresh).grid(row=0, column=4, padx=4, pady=0)
        self.wifi_status_label = ctk.CTkLabel(btn_frame, text="", text_color="gray", anchor="w")
        self.wifi_status_label.grid(row=1, column=0, columnspan=5, sticky="ew", pady=(6, 0))

        self._wifi_populate_local()
        self._wifi_set_status("Click Refresh to load device files.")
        self._wifi_update_sync_buttons_state()

    def _base_url(self) -> str:
        host = (self.host_entry.get() or "").strip() or "rdzsonde.local"
        if "://" in host:
            return host.rstrip("/") + "/"
        return "http://%s/" % host

    def _resolve_host(self) -> None:
        host = (self.host_entry.get() or "").strip() or "rdzsonde.local"
        try:
            import socket
            ip = socket.gethostbyname(host)
            self._resolved_ip = ip
            if hasattr(self, "ip_label"):
                self.ip_label.configure(text="IP: %s" % ip)
            self._wifi_set_status("Resolved %s → %s" % (host, ip))
        except Exception as e:
            self._resolved_ip = ""
            if hasattr(self, "ip_label"):
                self.ip_label.configure(text="")
            self._wifi_set_status("Resolve failed: %s" % e)

    def _test_connection(self) -> None:
        host = (self.host_entry.get() or "").strip() or "rdzsonde.local"
        self._wifi_set_status("Testing connection…")

        def work():
            return wifi_ops_mod.test_connection(host)

        def on_done(ok: bool, msg: str) -> None:
            if ok:
                self._wifi_set_status("Connected. %s" % (msg[:80] + "…" if len(msg) > 80 else msg))
            else:
                self._wifi_set_status("Failed: %s" % msg)

        _run_in_background(work, on_done, widget=self)

    def _wifi_set_status(self, text: str) -> None:
        if hasattr(self, "wifi_status_label"):
            self.wifi_status_label.configure(text=text)

    def _wifi_bin_offset_size(self) -> Tuple[int, Optional[int]]:
        """Return (file_offset, partition_size) for current _wifi_local_bin_path. partition_size None = whole file."""
        if not self._wifi_local_bin_path or not os.path.isfile(self._wifi_local_bin_path):
            return 0, None
        try:
            sz = os.path.getsize(self._wifi_local_bin_path)
        except OSError:
            return 0, None
        if sz > PARTITION_SPIFFS_SIZE:
            file_offset = PARTITION_SPIFFS_OFFSET - LFS_BACKUP_FLASH_BASE
            return file_offset, PARTITION_SPIFFS_SIZE
        return 0, None

    def _wifi_populate_local(self) -> None:
        if not hasattr(self, "wifi_local_tree"):
            return
        tree = self.wifi_local_tree
        tree.delete(*tree.get_children())

        if getattr(self, "_wifi_left_mode", "folder") == "bin" and self._wifi_local_bin_path:
            # Show contents of .bin image (LittleFS or SPIFFS, auto-detected)
            self._wifi_local_path_var.set(self._wifi_local_bin_path)
            tree.insert("", "end", iid="..", text="📁 ..", values=("",))
            off, size = self._wifi_bin_offset_size()
            try:
                entries, fs_type, recognized = list_fs_from_bin_auto(
                    self._wifi_local_bin_path, file_offset=off, partition_size=size
                )
                self._wifi_bin_fs_type = fs_type
                self._wifi_bin_off = off
                self._wifi_bin_size = size
                if not recognized:
                    self._wifi_set_status("No valid partition table or recognizable filesystem")
                    self._wifi_update_local_path_display()
                    self._wifi_update_sync_buttons_state()
                    return
            except Exception as e:
                self._wifi_set_status("Failed to read .bin: %s" % e)
                self._wifi_update_local_path_display()
                self._wifi_update_sync_buttons_state()
                return
            def _bin_sort_key(x: dict) -> tuple:
                name = x.get("name", "")
                ext = os.path.splitext(name)[1].lower()
                return (not x.get("dir", False), ext, name)
            for it in sorted(entries, key=_bin_sort_key):
                name = it.get("name", "?")
                is_dir = it.get("dir", False)
                sz = it.get("size", 0)
                iid = ("d:" if is_dir else "f:") + name
                size_str = (self._wifi_format_size_dots(sz) + " B") if sz else ""
                tree.insert("", "end", iid=iid, text=("📁 " if is_dir else "📄 ") + name, values=(size_str,))
            self._wifi_set_status("Showing %d entries from .bin image (%s)." % (len(entries), self._wifi_bin_fs_type))
        else:
            # Folder mode
            self._wifi_left_mode = "folder"
            self._wifi_local_bin_path = None
            self._wifi_bin_fs_type = getattr(self, "_wifi_bin_fs_type", "littlefs")
            self._wifi_bin_off = 0
            self._wifi_bin_size = None
            path = (self._wifi_local_path_var.get() or "").strip() or os.path.expanduser("~")
            if not os.path.isdir(path):
                path = os.path.expanduser("~")
            self._wifi_local_path = os.path.abspath(path)
            self._wifi_local_path_var.set(self._wifi_local_path)
            try:
                names = sorted(os.listdir(self._wifi_local_path))
            except OSError:
                self._wifi_update_local_path_display()
                self._wifi_update_sync_buttons_state()
                return
            dirs = [n for n in names if os.path.isdir(os.path.join(self._wifi_local_path, n))]
            files = [n for n in names if os.path.isfile(os.path.join(self._wifi_local_path, n))]
            if self._wifi_local_path != os.path.expanduser("~") and self._wifi_local_path != "/":
                tree.insert("", "end", iid="..", text="📁 ..", values=("",))
            for n in sorted(dirs):
                tree.insert("", "end", iid="d:" + n, text="📁 " + n, values=("",))
            for n in sorted(files, key=lambda fn: (os.path.splitext(fn)[1].lower(), fn)):
                try:
                    size = os.path.getsize(os.path.join(self._wifi_local_path, n))
                    tree.insert("", "end", iid="f:" + n, text="📄 " + n, values=(self._wifi_format_size_dots(size) + " B",))
                except OSError:
                    tree.insert("", "end", iid="f:" + n, text="📄 " + n, values=("",))

        self._wifi_update_local_path_display()
        self._wifi_update_sync_buttons_state()

    def _wifi_short_path_display(self, path: str) -> str:
        """Abbreviate path for display: show last component if long."""
        if not path:
            return ""
        if len(path) <= 40:
            return path
        base = os.path.basename(path)
        if len(base) > 38:
            return base[:35] + "..."
        return "…" + os.path.sep + base

    @staticmethod
    def _wifi_format_size_dots(n: int) -> str:
        """Format integer with '.' as thousands separator (e.g. 1234567 -> 1.234.567)."""
        s = str(max(0, n))
        if not s:
            return "0"
        parts = []
        for i in range(len(s), 0, -3):
            parts.append(s[max(0, i - 3) : i])
        return ".".join(reversed(parts))

    def _wifi_update_local_path_display(self) -> None:
        if not hasattr(self, "_wifi_local_path_label"):
            return
        if getattr(self, "_wifi_left_mode", "folder") == "bin" and self._wifi_local_bin_path:
            path = self._wifi_local_bin_path
        else:
            path = (self._wifi_local_path_var.get() or "").strip() or self._wifi_local_path
        self._wifi_local_path_label.configure(text=self._wifi_short_path_display(path))

    def _wifi_update_sync_buttons_state(self) -> None:
        if not hasattr(self, "_wifi_download_btn"):
            return
        if getattr(self, "_wifi_left_mode", "folder") == "bin":
            self._wifi_download_btn.configure(state="disabled")
        else:
            self._wifi_download_btn.configure(state="normal")

    def _wifi_local_path_show_tooltip(self, event: Any) -> None:
        if getattr(self, "_wifi_local_path_tooltip_after", None) is not None:
            self.after_cancel(self._wifi_local_path_tooltip_after)
            self._wifi_local_path_tooltip_after = None

        def show() -> None:
            self._wifi_local_path_tooltip_after = None
            path = (self._wifi_local_path_var.get() or "").strip() or getattr(self, "_wifi_local_path", "")
            if not path:
                return
            tip = tk.Toplevel(self.winfo_toplevel())
            tip.wm_overrideredirect(True)
            tip.attributes("-topmost", True)
            lbl = tk.Label(tip, text=path, background="#ffffe0", relief="solid", borderwidth=1, padx=4, pady=2)
            lbl.pack()
            x = self._wifi_local_path_label.winfo_rootx()
            y = self._wifi_local_path_label.winfo_rooty() + self._wifi_local_path_label.winfo_height() + 2
            tip.geometry("+%d+%d" % (x, y))
            self._wifi_local_path_tooltip = tip

        self._wifi_local_path_tooltip_after = self.after(400, show)

    def _wifi_local_path_hide_tooltip(self, event: Any) -> None:
        if getattr(self, "_wifi_local_path_tooltip_after", None) is not None:
            self.after_cancel(self._wifi_local_path_tooltip_after)
            self._wifi_local_path_tooltip_after = None
        if getattr(self, "_wifi_local_path_tooltip", None) is not None:
            self._wifi_local_path_tooltip.destroy()
            self._wifi_local_path_tooltip = None

    def _wifi_local_change(self) -> None:
        folder = filedialog.askdirectory(
            title="Choose local folder for TTGO files",
            initialdir=self._wifi_local_path if os.path.isdir(self._wifi_local_path) else self._initial_dir(),
        )
        if folder:
            self._wifi_left_mode = "folder"
            self._wifi_local_bin_path = None
            self._wifi_local_path = os.path.abspath(folder)
            self._wifi_local_path_var.set(self._wifi_local_path)
            self._wifi_populate_local()
            self._set_last_directory(folder)

    def _wifi_local_open_bin(self) -> None:
        path = filedialog.askopenfilename(
            title="Open .bin image – select backup or LittleFS image",
            initialdir=self._backup_initial_dir(),
            filetypes=[("Binary", "*.bin"), ("All", "*.*")],
        )
        if path:
            self._wifi_local_path = os.path.dirname(os.path.abspath(path))
            self._wifi_local_path_var.set(self._wifi_local_path)
            self._wifi_left_mode = "bin"
            self._wifi_local_bin_path = os.path.abspath(path)
            self._wifi_populate_local()

    def _wifi_local_on_double_click(self, event: Any) -> None:
        tree = self.wifi_local_tree
        sel = tree.identify_row(event.y)
        if not sel:
            return
        if sel == "..":
            if getattr(self, "_wifi_left_mode", "folder") == "bin":
                self._wifi_left_mode = "folder"
                self._wifi_local_bin_path = None
                self._wifi_local_path_var.set(self._wifi_local_path)
                self._wifi_populate_local()
            else:
                self._wifi_local_path = os.path.dirname(self._wifi_local_path)
                self._wifi_local_path_var.set(self._wifi_local_path)
                self._wifi_populate_local()
            return
        if isinstance(sel, str) and sel.startswith("d:"):
            name = sel[2:]
            self._wifi_local_path = os.path.join(self._wifi_local_path, name)
            self._wifi_local_path_var.set(self._wifi_local_path)
            self._wifi_populate_local()
            return
        if isinstance(sel, str) and sel.startswith("f:"):
            name = sel[2:]
            if name.lower().endswith(".bin"):
                self._wifi_left_mode = "bin"
                self._wifi_local_bin_path = os.path.join(self._wifi_local_path, name)
                self._wifi_populate_local()

    def _wifi_device_refresh(self) -> None:
        host = (self.host_entry.get() or "").strip() or "rdzsonde.local"
        user = (self.auth_user_entry.get() or "").strip() or None
        password = (self.auth_pass_entry.get() or "").strip() or None
        self._wifi_set_status("Loading…")
        self.wifi_device_tree.delete(*self.wifi_device_tree.get_children())
        self._wifi_device_entries = []

        def work():
            session, base_url, err = wifi_ops_mod.session_for_host(host, user, password)
            if err:
                return False, err
            entries, err = wifi_ops_mod.list_files(session, base_url, "")
            if err and not entries:
                return False, err
            return True, (entries, err)

        def on_done(ok: bool, msg: Any) -> None:
            if ok and isinstance(msg, tuple) and len(msg) == 2:
                entries, fallback_msg = msg
                if not isinstance(entries, list):
                    self._wifi_set_status("Error: invalid response")
                    return
                sorted_entries = sorted(
                    entries,
                    key=lambda it: (os.path.splitext(it.get("name", ""))[1].lower(), it.get("name", "")),
                )
                self._wifi_device_entries = sorted_entries
                for i, it in enumerate(sorted_entries):
                    name = it.get("name", "?")
                    is_dir = it.get("dir", 0)
                    size = it.get("size", 0)
                    size_str = (self._wifi_format_size_dots(size) + " B") if size else ""
                    prefix = "📁 " if is_dir else "📄 "
                    self.wifi_device_tree.insert("", "end", iid=str(i), text=prefix + name, values=(size_str,))
                self._wifi_set_status(fallback_msg if fallback_msg else "%d file(s) on device." % len(entries))
            else:
                self._wifi_set_status("Error: %s" % (msg if not ok else "Unknown"))

        _run_in_background(work, on_done, widget=self)

    def _wifi_download_selected(self) -> None:
        if getattr(self, "_wifi_left_mode", "folder") == "bin":
            self._wifi_set_status("Download is not available when viewing a .bin image.")
            return
        dest = (self._wifi_local_path_var.get() or "").strip() or self._wifi_local_path
        if not dest or not os.path.isdir(dest):
            self._wifi_set_status("Select a valid local folder.")
            return
        sel = self.wifi_device_tree.selection()
        if not sel:
            self._wifi_set_status("Select one or more device files to download.")
            return
        names = []
        for iid in sel:
            try:
                idx = int(iid)
                if 0 <= idx < len(self._wifi_device_entries):
                    names.append(self._wifi_device_entries[idx].get("name", ""))
            except ValueError:
                pass
        if not names:
            return
        host = (self.host_entry.get() or "").strip() or "rdzsonde.local"
        user = (self.auth_user_entry.get() or "").strip() or None
        password = (self.auth_pass_entry.get() or "").strip() or None
        total = len(names)
        self._wifi_set_status("Downloading %d file(s)…" % total)

        def work(progress_cb: Callable[[int, int], None]) -> Tuple[bool, str]:
            session, base_url, err = wifi_ops_mod.session_for_host(host, user, password)
            if err:
                return False, err
            saved = 0
            for name in names:
                if not name:
                    continue
                content, ferr = wifi_ops_mod.get_file(session, base_url, name)
                if ferr:
                    return False, "%s: %s" % (name, ferr)
                path = os.path.join(dest, name)
                with open(path, "wb") as f:
                    f.write(content)
                saved += 1
                progress_cb(saved, total)
            return True, "Downloaded %d file(s) to %s" % (saved, dest)

        def on_done(ok: bool, msg: str) -> None:
            self._wifi_set_status(msg)
            if ok:
                self._set_last_directory(dest)
                self._wifi_populate_local()  # refresh local folder so new files appear

        def on_progress(n: int, total: int) -> None:
            self._wifi_set_status("Downloading %d/%d…" % (n, total))

        _run_in_background(work, on_done, on_progress, widget=self)

    def _wifi_upload_selected(self) -> None:
        sel = self.wifi_local_tree.selection()
        if not sel:
            self._wifi_set_status("Select one or more local files to upload.")
            return
        files_to_upload: List[Tuple[str, str]] = []  # (path or name for bin, remote_name)
        from_bin = getattr(self, "_wifi_left_mode", "folder") == "bin" and self._wifi_local_bin_path
        if from_bin:
            for iid in sel:
                if iid == ".." or not isinstance(iid, str) or not iid.startswith("f:"):
                    continue
                name = iid[2:]
                files_to_upload.append((name, name))  # (name for read from bin, remote_name)
            bin_path = self._wifi_local_bin_path
            off, size = self._wifi_bin_offset_size()
        else:
            for iid in sel:
                if iid == ".." or not isinstance(iid, str):
                    continue
                if iid.startswith("f:"):
                    name = iid[2:]
                    path = os.path.join(self._wifi_local_path, name)
                    if os.path.isfile(path):
                        files_to_upload.append((path, name))
        if not files_to_upload:
            self._wifi_set_status("Select only files (not folders) to upload.")
            return
        host = (self.host_entry.get() or "").strip() or "rdzsonde.local"
        user = (self.auth_user_entry.get() or "").strip() or None
        password = (self.auth_pass_entry.get() or "").strip() or None
        total = len(files_to_upload)
        self._wifi_set_status("Uploading %d file(s)…" % total)

        def work(progress_cb: Callable[[int, int], None]) -> Tuple[bool, str]:
            session, base_url, err = wifi_ops_mod.session_for_host(host, user, password)
            if err:
                return False, err
            errs = []
            done = 0
            if from_bin:
                import tempfile
                fs_type = getattr(self, "_wifi_bin_fs_type", "littlefs")
                bin_off = getattr(self, "_wifi_bin_off", off)
                bin_size = getattr(self, "_wifi_bin_size", size)
                for path_or_name, name in files_to_upload:
                    try:
                        content = read_file_from_bin_auto(
                            bin_path, path_or_name, fs_type,
                            file_offset=bin_off, partition_size=bin_size,
                        )
                    except Exception as e:
                        errs.append("%s: %s" % (name, e))
                        done += 1
                        progress_cb(done, total)
                        continue
                    try:
                        tmp_fd, tmp_path = tempfile.mkstemp(prefix="wifi_upload_", suffix=".tmp")
                        try:
                            with os.fdopen(tmp_fd, "wb") as f:
                                f.write(content)
                            err = wifi_ops_mod.put_file(session, base_url, tmp_path, name)
                            if err:
                                errs.append("%s: %s" % (name, err))
                        finally:
                            try:
                                os.unlink(tmp_path)
                            except OSError:
                                pass
                    except Exception as e:
                        errs.append("%s: %s" % (name, e))
                    done += 1
                    progress_cb(done, total)
            else:
                for path, name in files_to_upload:
                    err = wifi_ops_mod.put_file(session, base_url, path, name)
                    if err:
                        errs.append("%s: %s" % (name, err))
                    done += 1
                    progress_cb(done, total)
            if errs:
                return False, "; ".join(errs[:3])
            return True, "Uploaded %d file(s)." % len(files_to_upload)

        def on_done(ok: bool, msg: str) -> None:
            self._wifi_set_status(msg)

        def on_progress(n: int, total: int) -> None:
            self._wifi_set_status("Uploading %d/%d…" % (n, total))

        _run_in_background(work, on_done, on_progress, widget=self)

    def _sd_refresh(self) -> None:
        self.sd_refresh_btn.configure(state="disabled")
        self.sd_placeholder.configure(text="Loading…")
        self.sd_placeholder.grid()
        self.sd_scroll.grid_remove()

        host = (self.host_entry.get() or "").strip() or "rdzsonde.local"
        user = (self.auth_user_entry.get() or "").strip() or None
        password = (self.auth_pass_entry.get() or "").strip() or None
        dir_path = self._sd_current_dir

        def work():
            session, base_url, err = wifi_ops_mod.session_for_host(host, user, password)
            if err:
                return False, err
            entries, err = sd_ops_mod.list_dir(session, base_url, dir_path)
            if err:
                return False, err
            return True, entries

        def on_done(ok: bool, msg: Any) -> None:
            self.sd_refresh_btn.configure(state="normal")
            if ok and isinstance(msg, list):
                self._sd_entries = msg
                self._sd_populate_list()
                self.sd_placeholder.grid_remove()
                self.sd_scroll.grid()
            else:
                self.sd_placeholder.configure(text="Error: %s" % (msg if not ok else "Unknown"))
                self.sd_placeholder.grid()
                self.sd_scroll.grid_remove()
        _run_in_background(work, on_done, widget=self)

    def _sd_sort_by_column(self, column: str) -> None:
        if column == getattr(self, "_sd_sort_column", "name"):
            self._sd_sort_reverse = not getattr(self, "_sd_sort_reverse", False)
        else:
            self._sd_sort_column = column
            self._sd_sort_reverse = False
        self._sd_populate_list()

    def _sd_populate_list(self) -> None:
        if not hasattr(self, "sd_tree"):
            return
        tree = self.sd_tree
        tree.delete(*tree.get_children())
        self._sd_iid_to_entry.clear()

        def _format_ts(ts: str) -> str:
            if not ts:
                return ""
            try:
                dt = datetime.datetime.fromisoformat(ts.replace("Z", "+00:00"))
            except Exception:
                return ts
            return dt.strftime("%Y-%m-%d %H:%M")

        def _format_size_dots(n: int) -> str:
            s = str(max(0, n))
            if not s:
                return "0"
            parts = []
            for i in range(len(s), 0, -3):
                parts.append(s[max(0, i - 3) : i])
            return ".".join(reversed(parts))

        def _sort_key_name(it: Any) -> tuple:
            return (0 if it.get("dir") else 1, str(it.get("name", "")).lower())

        def _sort_key_size(it: Any) -> tuple:
            return (0 if it.get("dir") else 1, int(it.get("size", 0)))

        def _sort_key_date(it: Any) -> tuple:
            ts = it.get("ts", "") or ""
            try:
                dt = datetime.datetime.fromisoformat(ts.replace("Z", "+00:00"))
                t = dt.timestamp()
            except Exception:
                t = 0.0
            return (0 if it.get("dir") else 1, t)

        sort_column = getattr(self, "_sd_sort_column", "date")
        sort_reverse = getattr(self, "_sd_sort_reverse", True)
        key = {"name": _sort_key_name, "size": _sort_key_size, "date": _sort_key_date}.get(
            sort_column, _sort_key_date
        )
        entries = sorted(self._sd_entries, key=key, reverse=sort_reverse)

        if self._sd_current_dir:
            tree.insert("", "end", iid="up", text="📁 ..", values=("", ""))
            self._sd_iid_to_entry["up"] = None

        for i, item in enumerate(entries):
            name = item.get("name", "?")
            is_dir = bool(item.get("dir"))
            size = int(item.get("size", 0)) if not is_dir else 0
            ts = item.get("ts", "")
            icon = "📁" if is_dir else "📄"
            display_name = icon + " " + (name + ("/" if is_dir else ""))
            size_text = "" if is_dir else _format_size_dots(size) + " B"
            ts_text = _format_ts(ts)
            iid = str(i)
            tree.insert("", "end", iid=iid, text=display_name, values=(size_text, ts_text))
            self._sd_iid_to_entry[iid] = item

    def _sd_on_double_click(self, event: Any) -> None:
        if not hasattr(self, "sd_tree"):
            return
        tree = self.sd_tree
        sel = tree.identify_row(event.y)
        if not sel:
            return
        if sel == "up":
            self._sd_go_up()
            return
        entry = self._sd_iid_to_entry.get(sel)
        if entry and entry.get("dir"):
            name = entry.get("name", "")
            self._sd_current_dir = (self._sd_current_dir + "/" + name).strip("/") if self._sd_current_dir else name
            self._sd_refresh()

    def _sd_go_up(self) -> None:
        if not self._sd_current_dir:
            return
        parts = self._sd_current_dir.rstrip("/").split("/")
        self._sd_current_dir = "/".join(parts[:-1]) if len(parts) > 1 else ""
        self._sd_refresh()

    def _sd_get_session_and_base(self) -> tuple[Any, str]:
        """Return (session, base_url) with auth if set. session has get(url)."""
        host = (self.host_entry.get() or "").strip() or "rdzsonde.local"
        user = (self.auth_user_entry.get() or "").strip() or None
        password = (self.auth_pass_entry.get() or "").strip() or None
        session, base_url, err = wifi_ops_mod.session_for_host(host, user, password)
        if err:
            raise RuntimeError(err)
        return session, base_url

    def _sd_collect_files_recursive(self, session: Any, base_url: str, dir_path: str) -> list[str]:
        """Return list of SD paths for all files under dir_path."""
        return sd_ops_mod.collect_files_recursive(session, base_url, dir_path)

    def _sd_download_all(self) -> None:
        initial = getattr(self, "_sd_download_initialdir", self._initial_dir)()
        dest = filedialog.askdirectory(
            title="Save SD card files – choose destination folder",
            initialdir=initial,
        )
        if not dest:
            return
        try:
            session, base_url = self._sd_get_session_and_base()
        except RuntimeError as e:
            messagebox.showerror("SD Download", str(e))
            return

        def work(progress_cb=None):
            files = sd_ops_mod.collect_files_recursive(session, base_url, self._sd_current_dir)
            saved, errs = sd_ops_mod.fetch_paths_to_dir(
                session, base_url, files, dest, progress_cb
            )
            if errs:
                return False, "Downloaded %s file(s). Errors: %s" % (saved, "; ".join(errs[:3]))
            return True, "Downloaded %s file(s) to %s" % (saved, dest)

        def on_progress(n: int, total: int) -> None:
            self.sd_progress.set(n / total if total else 0)
            self.sd_progress_label.configure(text="Downloading %d / %d" % (n, total))

        def on_done(ok: bool, msg: str) -> None:
            self.sd_progress.set(1)
            self.sd_progress_label.configure(text="Done." if ok else "Failed.")
            if ok:
                self._set_last_directory(dest)
                getattr(self, "_sd_set_download_dir", lambda _: None)(dest)
                messagebox.showinfo("SD Download", msg)
            else:
                messagebox.showerror("SD Download", msg)

        self.sd_progress.set(0)
        self.sd_progress_label.configure(text="Starting…")
        _run_in_background(work, on_done, on_progress, widget=self)

    def _sd_download_selected(self) -> None:
        if hasattr(self, "sd_tree"):
            selected = [
                self._sd_iid_to_entry[iid]
                for iid in self.sd_tree.selection()
                if iid in self._sd_iid_to_entry and self._sd_iid_to_entry[iid] is not None
            ]
        else:
            selected = []
        if not selected:
            messagebox.showwarning("SD Download", "Select one or more items (files or folders) to download.")
            return
        initial = getattr(self, "_sd_download_initialdir", self._initial_dir)()
        dest = filedialog.askdirectory(
            title="Save SD card files – choose destination folder",
            initialdir=initial,
        )
        if not dest:
            return
        try:
            session, base_url = self._sd_get_session_and_base()
        except RuntimeError as e:
            messagebox.showerror("SD Download", str(e))
            return

        def work(progress_cb=None):
            all_paths: list[str] = []
            for entry in selected:
                name = entry.get("name", "")
                if not name:
                    continue
                full = (self._sd_current_dir + "/" + name).strip("/") if self._sd_current_dir else name
                if entry.get("dir"):
                    all_paths.extend(sd_ops_mod.collect_files_recursive(session, base_url, full))
                else:
                    all_paths.append(full)
            saved, errs = sd_ops_mod.fetch_paths_to_dir(
                session, base_url, all_paths, dest, progress_cb
            )
            if errs:
                return False, "Downloaded %s file(s). Errors: %s" % (saved, "; ".join(errs[:3]))
            return True, "Downloaded %s file(s) to %s" % (saved, dest)

        def on_progress(n: int, total: int) -> None:
            self.sd_progress.set(n / total if total else 0)
            self.sd_progress_label.configure(text="Downloading %d / %d" % (n, total))

        def on_done(ok: bool, msg: str) -> None:
            self.sd_progress.set(1)
            self.sd_progress_label.configure(text="Done." if ok else "Failed.")
            if ok:
                self._set_last_directory(dest)
                getattr(self, "_sd_set_download_dir", lambda _: None)(dest)
                messagebox.showinfo("SD Download", msg)
            else:
                messagebox.showerror("SD Download", msg)

        self.sd_progress.set(0)
        self.sd_progress_label.configure(text="Starting…")
        _run_in_background(work, on_done, on_progress, widget=self)


class SDWiFiFrame(FilesWiFiFrame):
    """SD card download via Wi-Fi in its own tab."""

    def __init__(self, master, app: Optional["App"] = None, **kwargs):
        ContentFrame.__init__(self, master, "SD (via WiFi)", app=app, **kwargs)
        self.grid_columnconfigure(0, weight=1)

        row, self.host_entry, self.ip_label, self.auth_user_entry, self.auth_pass_entry = _make_host_auth_row(
            self, self.app, 0, self._resolve_host, self._test_connection, columnspan=1
        )

        # SD card download: browse folders (device /files.json style), then download all or selected folder(s)
        ctk.CTkLabel(self, text="SD card download", font=ctk.CTkFont(weight="bold")).grid(
            row=row, column=0, sticky="w", padx=20, pady=(18, 6))
        row += 1
        sd_desc = ctk.CTkLabel(
            self,
            text="Browse folders (like device list directory). Select items (click or Shift+click), double-click folder to open.",
            text_color="gray",
        )
        sd_desc.grid(row=row, column=0, sticky="w", padx=20, pady=(0, 4))
        row += 1
        sd_frame = ctk.CTkFrame(self, fg_color="transparent")
        sd_frame.grid(row=row, column=0, sticky="nsew", padx=20, pady=4)
        sd_frame.grid_columnconfigure(0, weight=1)
        self.grid_rowconfigure(row, weight=1)
        self._sd_entries = []
        self._sd_iid_to_entry: Dict[str, Any] = {}  # iid -> entry dict or None for ".."
        self._sd_sort_column = "date"
        self._sd_sort_reverse = True  # newest first by default
        self.sd_placeholder = ctk.CTkLabel(
            sd_frame,
            text="Click Refresh list to load SD root (requires device connected via Wi-Fi).",
            text_color="gray",
        )
        self.sd_placeholder.grid(row=0, column=0, sticky="nsew", pady=(0, 6))
        # Treeview + scrollbar in a tk.Frame for native scrolling (e.g. macOS trackpad)
        self.sd_scroll = tk.Frame(sd_frame, height=120)
        self.sd_scroll.grid(row=0, column=0, columnspan=2, sticky="nsew", pady=(0, 6))
        self.sd_scroll.grid_remove()
        self.sd_scroll.grid_propagate(False)
        self.sd_tree = ttk.Treeview(
            self.sd_scroll,
            columns=("size", "date"),
            show="tree headings",
            selectmode="extended",
            height=8,
        )
        self.sd_tree.heading("#0", text="Name", command=lambda: self._sd_sort_by_column("name"))
        self.sd_tree.heading("size", text="Size", command=lambda: self._sd_sort_by_column("size"))
        self.sd_tree.heading("date", text="Date", command=lambda: self._sd_sort_by_column("date"))
        self.sd_tree.column("#0", width=220, minwidth=100)
        self.sd_tree.column("size", width=80, minwidth=50, anchor="e")
        self.sd_tree.column("date", width=120, minwidth=80)
        sd_tree_yscroll = ttk.Scrollbar(self.sd_scroll, orient="vertical", command=self.sd_tree.yview)
        self.sd_tree.configure(yscrollcommand=sd_tree_yscroll.set)
        self.sd_tree.pack(side="left", fill="both", expand=True)
        sd_tree_yscroll.pack(side="right", fill="y")
        self.sd_tree.bind("<Double-1>", self._sd_on_double_click)
        sd_frame.grid_rowconfigure(0, weight=1)
        sd_dest_row = ctk.CTkFrame(sd_frame, fg_color="transparent")
        sd_dest_row.grid(row=1, column=0, columnspan=2, sticky="ew", pady=(0, 4))
        sd_dest_row.grid_columnconfigure(1, weight=1)
        ctk.CTkLabel(sd_dest_row, text="Download to:").grid(row=0, column=0, sticky="w", padx=(0, 8), pady=0)
        self.sd_download_to_var = tk.StringVar(value="(not set)")
        self.sd_download_to_label = ctk.CTkLabel(sd_dest_row, textvariable=self.sd_download_to_var, text_color="gray", anchor="w")
        self.sd_download_to_label.grid(row=0, column=1, sticky="ew", padx=(0, 10), pady=0)
        ctk.CTkButton(sd_dest_row, text="Change", width=80, command=self._sd_change_download_dir).grid(row=0, column=2, padx=0, pady=0)
        sd_btns = ctk.CTkFrame(sd_frame, fg_color="transparent")
        sd_btns.grid(row=2, column=0, columnspan=2, sticky="w", pady=(0, 4))
        self.sd_refresh_btn = ctk.CTkButton(sd_btns, text="Refresh list", width=100, command=self._sd_refresh)
        self.sd_refresh_btn.grid(row=0, column=0, padx=(0, 10), pady=0)
        ctk.CTkButton(sd_btns, text="Download all", width=120, command=self._sd_download_all).grid(row=0, column=1, padx=(10, 0), pady=0)
        ctk.CTkButton(sd_btns, text="Download selected", width=140, command=self._sd_download_selected).grid(row=0, column=2, padx=(10, 0), pady=0)
        self._sd_current_dir = ""
        self.sd_progress = ctk.CTkProgressBar(sd_frame, height=8, progress_color=("gray70", "gray35"))
        self.sd_progress.grid(row=3, column=0, columnspan=2, sticky="ew", pady=(4, 0))
        self.sd_progress.set(0)
        self.sd_progress_label = ctk.CTkLabel(sd_frame, text="", text_color="gray", height=0)
        self.sd_progress_label.grid(row=4, column=0, columnspan=2, sticky="w", pady=(2, 0))
        self._sd_update_download_to_display()

    def _sd_update_download_to_display(self) -> None:
        path = (self.app.settings if self.app else {}).get("last_sd_download_dir", "").strip()
        self.sd_download_to_var.set(path if path else "(not set)")

    def _sd_change_download_dir(self) -> None:
        initial = (self.app.settings if self.app else {}).get("last_sd_download_dir", "").strip() or os.path.expanduser("~")
        d = filedialog.askdirectory(
            title="SD download – choose default destination folder",
            initialdir=initial if os.path.isdir(initial) else os.path.expanduser("~"),
        )
        if d:
            if self.app:
                self.app.settings["last_sd_download_dir"] = d
                settings_mod.save(self.app.settings)
            self._sd_update_download_to_display()

    def _sd_download_initialdir(self) -> str:
        path = (self.app.settings if self.app else {}).get("last_sd_download_dir", "").strip()
        return path if path and os.path.isdir(path) else os.path.expanduser("~")

    def _sd_set_download_dir(self, dest: str) -> None:
        if self.app:
            self.app.settings["last_sd_download_dir"] = dest
            settings_mod.save(self.app.settings)
        self._sd_update_download_to_display()

    def _test_connection(self) -> None:
        host = (self.host_entry.get() or "").strip() or "rdzsonde.local"
        self.sd_placeholder.configure(
            text="Testing connection to %s…" % host,
            text_color="gray",
        )
        self.sd_placeholder.grid()
        self.sd_scroll.grid_remove()

        def work():
            return wifi_ops_mod.test_connection(host)

        def on_done(ok: bool, msg: str) -> None:
            if ok:
                text = "Connected to %s" % host
                color = "green"
            else:
                text = "Connection failed: %s" % msg
                color = "red"
            self.sd_placeholder.configure(text=text, text_color=color)
            self.sd_placeholder.grid()
            self.sd_scroll.grid_remove()

        _run_in_background(work, on_done, widget=self)


class ImprovDialog(ctk.CTkToplevel):
    """IMPROV WiFi setup: get device info, scan networks, connect to selected network."""

    def __init__(
        self,
        parent: Any,
        port: str,
        baud: int,
        serial_frame: "SerialLogFrame",
        was_connected: bool,
        **kwargs: Any,
    ) -> None:
        super().__init__(parent, **kwargs)
        self.title("Setup via IMPROV")
        self.port = port
        self.baud = baud
        self.serial_frame = serial_frame
        self.was_connected = was_connected
        self._ser: Any = None
        self._improv_thread: Optional[threading.Thread] = None
        self._device_info: Optional[List[str]] = None
        self._networks: List[Tuple[str, str, str]] = []
        self._connect_in_progress = False
        self.geometry("480x420")
        self.grid_columnconfigure(0, weight=1)
        self.grid_rowconfigure(2, weight=1)

        # Status
        self.status_var = tk.StringVar(value="Opening port…")
        ctk.CTkLabel(self, textvariable=self.status_var, anchor="w").grid(
            row=0, column=0, sticky="ew", padx=15, pady=(15, 5)
        )
        # Device info (name / version)
        self.info_var = tk.StringVar(value="")
        ctk.CTkLabel(self, textvariable=self.info_var, anchor="w").grid(
            row=1, column=0, sticky="ew", padx=15, pady=(0, 5)
        )
        # Networks list
        list_frame = ctk.CTkFrame(self, fg_color="transparent")
        list_frame.grid(row=2, column=0, sticky="nsew", padx=15, pady=5)
        list_frame.grid_columnconfigure(0, weight=1)
        list_frame.grid_rowconfigure(1, weight=1)
        ctk.CTkLabel(list_frame, text="Networks").grid(row=0, column=0, sticky="w")
        self.network_listbox = tk.Listbox(
            list_frame, height=8, selectmode=tk.SINGLE, font=("TkDefaultFont", 10)
        )
        self.network_listbox.grid(row=1, column=0, sticky="nsew", pady=(2, 5))
        scroll = ttk.Scrollbar(list_frame, orient=tk.VERTICAL, command=self.network_listbox.yview)
        scroll.grid(row=1, column=1, sticky="ns")
        self.network_listbox.configure(yscrollcommand=scroll.set)
        # Custom SSID
        ssid_frame = ctk.CTkFrame(self, fg_color="transparent")
        ssid_frame.grid(row=3, column=0, sticky="ew", padx=15, pady=2)
        ssid_frame.grid_columnconfigure(1, weight=1)
        ctk.CTkLabel(ssid_frame, text="Custom SSID:").grid(row=0, column=0, padx=(0, 8), pady=2)
        self.custom_ssid_entry = ctk.CTkEntry(ssid_frame, placeholder_text="or type SSID")
        self.custom_ssid_entry.grid(row=0, column=1, sticky="ew", pady=2)
        # Password
        pw_frame = ctk.CTkFrame(self, fg_color="transparent")
        pw_frame.grid(row=4, column=0, sticky="ew", padx=15, pady=2)
        pw_frame.grid_columnconfigure(1, weight=1)
        ctk.CTkLabel(pw_frame, text="Password:").grid(row=0, column=0, padx=(0, 8), pady=2)
        self.password_entry = ctk.CTkEntry(pw_frame, placeholder_text="WiFi password", show="*")
        self.password_entry.grid(row=0, column=1, sticky="ew", pady=2)
        # Buttons
        btn_frame = ctk.CTkFrame(self, fg_color="transparent")
        btn_frame.grid(row=5, column=0, sticky="ew", padx=15, pady=(10, 15))
        btn_frame.grid_columnconfigure(0, weight=1)
        self.connect_btn = ctk.CTkButton(
            btn_frame, text="Connect", width=100, command=self._on_connect
        )
        self.connect_btn.grid(row=0, column=0, padx=(0, 10))
        ctk.CTkButton(btn_frame, text="Close", width=80, command=self._on_close).grid(
            row=0, column=1
        )
        self.connect_btn.configure(state="disabled")

        self.protocol("WM_DELETE_WINDOW", self._on_close)
        self.after(50, self._start_improv_flow)

    def _set_status(self, text: str) -> None:
        self.status_var.set(text)

    def _set_info(self, text: str) -> None:
        self.info_var.set(text)

    def _start_improv_flow(self) -> None:
        def work() -> None:
            try:
                import serial
                ser = serial.Serial(self.port, self.baud, timeout=1.0)
            except Exception as e:
                err_msg = str(e)
                self.after(0, lambda msg=err_msg: self._on_open_failed(msg))
                return
            self._ser = ser
            self.after(0, lambda: self._set_status("Requesting device info…"))
            info = improv_serial_mod.rpc_get_info(ser, timeout_sec=3.0)
            if info is None:
                self.after(0, lambda: self._on_get_info_failed())
                return
            self._device_info = info
            self.after(0, lambda: self._set_status("Scanning WiFi…"))
            self.after(0, lambda: self._set_info("%s  %s" % (info[0] if len(info) > 0 else "", info[1] if len(info) > 1 else "")))
            networks = improv_serial_mod.rpc_wifi_scan(ser, timeout_sec=10.0)
            self._networks = networks
            self.after(0, lambda: self._on_scan_done(networks))

        self._improv_thread = threading.Thread(target=work, daemon=True)
        self._improv_thread.start()

    def _on_open_failed(self, msg: str) -> None:
        if not self.winfo_exists():
            return
        self._set_status("Could not open port: %s" % msg)
        self.connect_btn.configure(state="disabled")

    def _on_get_info_failed(self) -> None:
        if not self.winfo_exists():
            return
        self._set_status("Could not get device info (timeout or invalid response).")
        if self._ser:
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None
        self.connect_btn.configure(state="disabled")

    def _on_scan_done(self, networks: List[Tuple[str, str, str]]) -> None:
        if not self.winfo_exists():
            return
        self._set_status("Select a network or enter custom SSID and password, then Connect.")
        self.network_listbox.delete(0, tk.END)
        for ssid, rssi, secured in networks:
            self.network_listbox.insert(tk.END, "%s  (%s, %s)" % (ssid, rssi, secured))
        if not networks:
            self._set_status("No networks found. Enter custom SSID and password, then Connect.")
        self.connect_btn.configure(state="normal")

    def _get_selected_ssid(self) -> Optional[str]:
        custom = (self.custom_ssid_entry.get() or "").strip()
        if custom:
            return custom
        sel = self.network_listbox.curselection()
        if not sel:
            return None
        idx = int(sel[0])
        if 0 <= idx < len(self._networks):
            return self._networks[idx][0]
        text = self.network_listbox.get(sel[0])
        if "  (" in text:
            return text.split("  (")[0].strip()
        return text.strip()

    def _on_connect(self) -> None:
        ssid = self._get_selected_ssid()
        if not ssid:
            messagebox.showwarning("IMPROV", "Select a network from the list or enter a custom SSID.")
            return
        password = (self.password_entry.get() or "").strip()
        if self._connect_in_progress or self._ser is None:
            return
        self._connect_in_progress = True
        self.connect_btn.configure(state="disabled")
        self._set_status("Connecting…")

        def work() -> None:
            try:
                url = improv_serial_mod.rpc_send_wifi(self._ser, ssid, password, timeout_sec=15.0)
                self.after(0, lambda u=url: self._on_connect_done(u))
            except Exception as e:
                err_msg = str(e)
                self.after(0, lambda msg=err_msg: self._on_connect_done(None, msg))
            finally:
                self.after(0, lambda: self._set_connect_done())

        threading.Thread(target=work, daemon=True).start()

    def _set_connect_done(self) -> None:
        if not self.winfo_exists():
            return
        self._connect_in_progress = False
        self.connect_btn.configure(state="normal")

    def _on_connect_done(self, url: Optional[str], error: Optional[str] = None) -> None:
        if not self.winfo_exists():
            return
        if error:
            self._set_status("Connection failed: %s" % error)
            return
        if url:
            self._set_status("Connected. URL: %s" % url)
            messagebox.showinfo("IMPROV", "Connected. Device URL: %s" % url)
        else:
            self._set_status("Connection failed (wrong password or no response).")
            messagebox.showerror("IMPROV", "Connection failed. Check password and try again.")

    def _on_close(self) -> None:
        if self._ser and getattr(self._ser, "is_open", False):
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None
        if self.was_connected:
            messagebox.showinfo(
                "Setup via IMPROV",
                "Serial was disconnected for IMPROV. Reconnect from the Serial tab if needed.",
            )
        self.destroy()


class SerialLogFrame(ContentFrame):
    def __init__(self, master, app: Optional["App"] = None, **kwargs):
        super().__init__(master, "Serial", app=app, **kwargs)
        self.grid_columnconfigure(0, weight=1)
        self.grid_rowconfigure(1, weight=1)
        self._serial: Optional[Any] = None
        self._serial_thread: Optional[threading.Thread] = None
        self._serial_stop = threading.Event()
        self._serial_queue: queue.Queue[str] = queue.Queue()

        row = 0
        dev_frame = ctk.CTkFrame(self, fg_color="transparent")
        dev_frame.grid(row=row, column=0, sticky="ew", padx=20, pady=(20, 10))
        next_col = self._make_port_combo_row(dev_frame)
        ctk.CTkLabel(dev_frame, text="Baud:").grid(row=0, column=next_col, padx=(20, 8), pady=5)
        self.baud_combo = ctk.CTkComboBox(
            dev_frame, values=["115200", "9600", "19200", "38400", "57600", "230400", "460800", "921600"],
            width=100,
        )
        saved_baud = (app.settings if app else {}).get("baud_rate", "115200")
        self.baud_combo.set(saved_baud if saved_baud in self.baud_combo.cget("values") else "115200")
        self.baud_combo.grid(row=0, column=next_col + 1, padx=0, pady=5, sticky="w")
        self.connect_btn = ctk.CTkButton(dev_frame, text="Connect", width=90, command=self._toggle_connect)
        self.connect_btn.grid(row=0, column=next_col + 2, padx=(10, 0), pady=5)
        row += 1

        self.log_text = ctk.CTkTextbox(self, font=ctk.CTkFont(family="monospace"))
        self.log_text.grid(row=row, column=0, sticky="nsew", padx=20, pady=10)
        self._serial_log_tk = getattr(self.log_text, "_textbox", None)
        self.log_text.insert("1.0", "Select a port and connect to see serial output.\n")
        _configure_ansi_tags_for_log(self.log_text)
        row += 1

        bot = ctk.CTkFrame(self, fg_color="transparent")
        bot.grid(row=row, column=0, sticky="ew", padx=20, pady=(0, 20))
        bot.grid_columnconfigure(0, weight=1)
        ctk.CTkButton(bot, text="Setup via IMPROV", width=120, command=self._setup_improv).grid(row=0, column=0, padx=(0, 10), pady=0)
        ctk.CTkButton(bot, text="Save to file", width=100, command=self._save_log).grid(row=0, column=1, padx=(0, 10), pady=0)
        ctk.CTkButton(bot, text="Clear", width=80, command=self._clear_log).grid(row=0, column=2, padx=10, pady=0)
        ctk.CTkButton(bot, text="Copy", width=80, command=self._copy_log).grid(row=0, column=3, padx=0, pady=0)

    def _setup_improv(self) -> None:
        """Open IMPROV dialog: get device info, scan WiFi, connect to selected network."""
        port = self.port_combo.get()
        if not port or port == "Auto" or port.startswith("("):
            messagebox.showwarning("Setup via IMPROV", "Select a specific serial port (not Auto).")
            return
        try:
            baud = int(self.baud_combo.get())
        except (TypeError, ValueError):
            baud = 115200
        was_connected = bool(self._serial and self._serial.is_open)
        if was_connected:
            self._serial_stop.set()
            try:
                if self._serial.is_open:
                    self._serial.close()
            except Exception:
                pass
            self._serial = None
            self._serial_thread = None
            self.connect_btn.configure(text="Connect")
        dialog = ImprovDialog(self.winfo_toplevel(), port=port, baud=baud, serial_frame=self, was_connected=was_connected)
        dialog.transient(self.winfo_toplevel())
        dialog.grab_set()
        dialog.focus_force()

    def _append_log(self, text: str) -> None:
        """Append text to serial log with ANSI color parsing."""
        segments = _parse_ansi_to_segments(text)
        tb = self._serial_log_tk or self.log_text

        def do():
            tw = self._serial_log_tk or self.log_text
            at_bottom = tw.yview()[1] >= 0.999
            self.log_text.configure(state="normal")
            for seg_text, tag in segments:
                if tag:
                    tb.insert("end", seg_text, tag)
                else:
                    tb.insert("end", seg_text)
            if at_bottom:
                self.log_text.see("end")
            self.log_text.configure(state="disabled")
        self.after(0, do)

    def _strip_ansi(self, s: str) -> str:
        import re
        return re.sub(r"\x1b\[[0-9;]*m", "", s)

    def _save_log(self) -> None:
        path = filedialog.asksaveasfilename(
            title="Save serial log – choose file",
            initialdir=self._initial_dir(),
            defaultextension=".txt",
            filetypes=[("Text", "*.txt"), ("All", "*.*")],
        )
        if not path:
            return
        try:
            content = self.log_text.get("1.0", "end")
            with open(path, "w", encoding="utf-8", errors="replace") as f:
                f.write(content)
            self._set_last_directory(path)
            messagebox.showinfo("Save", "Log saved.")
        except Exception as e:
            messagebox.showerror("Save", str(e))

    def _clear_log(self) -> None:
        self.log_text.configure(state="normal")
        self.log_text.delete("1.0", "end")
        self.log_text.configure(state="disabled")

    def _copy_log(self) -> None:
        try:
            self.clipboard_clear()
            self.clipboard_append(self.log_text.get("1.0", "end"))
            messagebox.showinfo("Copy", "Log copied to clipboard.")
        except Exception:
            pass

    def _toggle_connect(self) -> None:
        if self._serial and self._serial.is_open:
            self._serial_stop.set()
            try:
                self._serial.close()
            except Exception:
                pass
            self._serial = None
            self._serial_thread = None
            self.connect_btn.configure(text="Connect")
            return
        port = self.port_combo.get()
        if not port or port == "Auto" or port.startswith("("):
            messagebox.showwarning("Serial", "Select a specific port (not Auto).")
            return
        try:
            baud = int(self.baud_combo.get())
        except (TypeError, ValueError):
            baud = 115200
        try:
            import serial
            self._serial = serial.Serial(port, baud, timeout=0.5)
            self._serial_stop.clear()
            self._serial_thread = threading.Thread(target=self._serial_reader_loop, daemon=True)
            self._serial_thread.start()
            self.connect_btn.configure(text="Disconnect")
            self._append_log("\n[Connected to %s @ %s]\n" % (port, baud))
            self.after(200, self._drain_serial_queue)
        except Exception as e:
            messagebox.showerror("Serial", "Could not open port: %s" % e)

    def _on_serial_disconnected(self) -> None:
        """Called on main thread when the serial reader loop exits (error or port closed)."""
        if not self.winfo_exists():
            return
        if self._serial is not None:
            self._serial_stop.set()
            try:
                if self._serial.is_open:
                    self._serial.close()
            except Exception:
                pass
            self._serial = None
            self._serial_thread = None
        self.connect_btn.configure(text="Connect")

    def _serial_reader_loop(self) -> None:
        try:
            ser = self._serial
            if not ser or not ser.is_open:
                return
            while not self._serial_stop.is_set() and ser.is_open:
                line = ser.readline()
                if line:
                    try:
                        text = line.decode("utf-8", errors="replace")
                    except Exception:
                        text = line.decode("latin-1", errors="replace")
                    self._serial_queue.put(text)
        except Exception as e:
            errno = getattr(e, "errno", None)
            msg = str(e).lower()
            if errno != 9 and "bad file descriptor" not in msg:
                self._serial_queue.put("\n[Error: %s]\n" % e)
        finally:
            self._serial_queue.put("\n[Disconnected]\n")
            self.after(0, self._on_serial_disconnected)

    def _drain_serial_queue(self) -> None:
        """Drain serial queue and append to log on main thread; reschedule while connected."""
        chunks: list[str] = []
        try:
            while True:
                chunks.append(self._serial_queue.get_nowait())
        except queue.Empty:
            pass
        if chunks:
            text = "".join(chunks)
            segments = _parse_ansi_to_segments(text)
            tb = self._serial_log_tk or self.log_text

            def do():
                tw = self._serial_log_tk or self.log_text
                at_bottom = tw.yview()[1] >= 0.999
                self.log_text.configure(state="normal")
                for seg_text, tag in segments:
                    if tag:
                        tb.insert("end", seg_text, tag)
                    else:
                        tb.insert("end", seg_text)
                # Enforce scroll-back buffer line limit (min 100 lines)
                try:
                    raw = self.app.settings.get("serial_log_buffer_lines", settings_mod.DEFAULT_SERIAL_LOG_LINES)
                    limit = max(100, int(raw) if isinstance(raw, int) else int(raw) if raw else 100)
                except (TypeError, ValueError):
                    limit = max(100, getattr(settings_mod, "DEFAULT_SERIAL_LOG_LINES", 10000))
                tk_tb = self._serial_log_tk or getattr(self.log_text, "_textbox", None)
                if tk_tb is not None:
                    line_count = int(tk_tb.index("end").split(".")[0])
                    if line_count > limit:
                        self.log_text.configure(state="normal")
                        tk_tb.delete("1.0", "%d.0" % (line_count - limit + 1))
                        self.log_text.configure(state="disabled")
                if at_bottom:
                    self.log_text.see("end")
                self.log_text.configure(state="disabled")
            self.after(0, do)
        # Only keep polling while the connection is active and the widget still exists.
        if (
            self.winfo_exists()
            and self._serial is not None
            and getattr(self._serial, "is_open", False)
            and not self._serial_stop.is_set()
        ):
            self.after(200, self._drain_serial_queue)

    def show(self) -> None:
        super().show()

    def hide(self) -> None:
        if self._serial and self._serial.is_open:
            self._serial_stop.set()
            try:
                self._serial.close()
            except Exception:
                pass
            self._serial = None
            self._serial_thread = None
        super().hide()


class SettingsFrame(ContentFrame):
    def __init__(self, master, app: Optional["App"] = None, **kwargs):
        super().__init__(master, "Settings", app=app, **kwargs)
        self.grid_columnconfigure(0, weight=0)   # labels
        self.grid_columnconfigure(1, weight=1)  # entries
        self.grid_columnconfigure(2, weight=0)  # buttons

        LABEL_W = 200  # fixed width so all inputs align
        ENTRY_MIN_W = 320

        row = 0
        ctk.CTkLabel(self, text="Backup folder:", width=LABEL_W, anchor="w").grid(
            row=row, column=0, sticky="w", padx=(20, 10), pady=(20, 4))
        self.backup_folder_var = ctk.StringVar(value="")
        self.backup_entry = ctk.CTkEntry(
            self, textvariable=self.backup_folder_var, placeholder_text="e.g. ~/rdzTTGOsonde/backups", width=ENTRY_MIN_W
        )
        self.backup_entry.grid(row=row, column=1, sticky="ew", padx=0, pady=(20, 4))
        ctk.CTkButton(self, text="Browse…", width=80, command=self._browse_backup).grid(
            row=row, column=2, padx=(10, 20), pady=(20, 4))
        row += 1

        ctk.CTkLabel(self, text="Firmware server URL:", width=LABEL_W, anchor="w").grid(
            row=row, column=0, sticky="w", padx=(20, 10), pady=4)
        self.download_url_var = ctk.StringVar(value="")
        ctk.CTkEntry(self, textvariable=self.download_url_var, placeholder_text="https://rdzsonde.org", width=ENTRY_MIN_W).grid(
            row=row, column=1, sticky="ew", padx=0, pady=4)
        row += 1

        ctk.CTkLabel(self, text="Serial", font=ctk.CTkFont(weight="bold")).grid(
            row=row, column=0, columnspan=3, sticky="w", padx=20, pady=(16, 8))
        row += 1
        ctk.CTkLabel(self, text="Scroll-back buffer (lines):", width=LABEL_W, anchor="w").grid(
            row=row, column=0, sticky="w", padx=(20, 10), pady=4)
        self.serial_log_lines_var = ctk.StringVar(value="10000")
        ctk.CTkEntry(self, textvariable=self.serial_log_lines_var, width=100, placeholder_text="10000").grid(
            row=row, column=1, sticky="w", padx=0, pady=4)
        row += 1

        ctk.CTkLabel(self, text="ESPtool advanced settings", font=ctk.CTkFont(weight="bold")).grid(
            row=row, column=0, columnspan=3, sticky="w", padx=20, pady=(16, 8))
        row += 1

        self.baud_read_var = ctk.StringVar(value="921600")
        self.no_stub_read_var = ctk.BooleanVar(value=True)
        ctk.CTkLabel(self, text="read-flash baud rate:", width=LABEL_W, anchor="w").grid(
            row=row, column=0, sticky="w", padx=(20, 10), pady=4)
        ctk.CTkComboBox(self, values=["921600", "115200", "460800"], variable=self.baud_read_var, width=120).grid(
            row=row, column=1, sticky="w", padx=0, pady=4)
        ctk.CTkCheckBox(self, text="Use --no-stub", variable=self.no_stub_read_var).grid(
            row=row, column=2, sticky="w", padx=(10, 20), pady=4)
        row += 1

        self.baud_write_var = ctk.StringVar(value="921600")
        self.no_stub_write_var = ctk.BooleanVar(value=False)
        ctk.CTkLabel(self, text="write-flash baud rate:", width=LABEL_W, anchor="w").grid(
            row=row, column=0, sticky="w", padx=(20, 10), pady=4)
        ctk.CTkComboBox(self, values=["921600", "460800", "115200"], variable=self.baud_write_var, width=120).grid(
            row=row, column=1, sticky="w", padx=0, pady=4)
        ctk.CTkCheckBox(self, text="Use --no-stub", variable=self.no_stub_write_var).grid(
            row=row, column=2, sticky="w", padx=(10, 20), pady=4)
        row += 1
        self.grid_rowconfigure(row, weight=1)

    def _browse_backup(self) -> None:
        d = filedialog.askdirectory(
            title="Settings – choose backup folder for device backups",
            initialdir=self._initial_dir(),
        )
        if d:
            self.backup_folder_var.set(d)
            self._save_from_ui()

    def load_from_settings(self, s: Dict[str, Any]) -> None:
        self.backup_folder_var.set(s.get("backup_folder", settings_mod.DEFAULT_BACKUP_FOLDER))
        self.download_url_var.set(s.get("download_url", settings_mod.DEFAULT_DOWNLOAD_URL))
        self.serial_log_lines_var.set(str(s.get("serial_log_buffer_lines", settings_mod.DEFAULT_SERIAL_LOG_LINES)))
        self.baud_read_var.set(s.get("baud_rate_read", settings_mod.DEFAULT_BAUD_READ))
        self.baud_write_var.set(s.get("baud_rate_write", settings_mod.DEFAULT_BAUD_WRITE))
        self.no_stub_read_var.set(bool(s.get("no_stub_read", True)))
        self.no_stub_write_var.set(bool(s.get("no_stub_write", False)))

    def _save_from_ui(self) -> None:
        if not self.app:
            return
        try:
            n = max(100, int(self.serial_log_lines_var.get().strip() or "10000"))
        except ValueError:
            n = max(100, getattr(settings_mod, "DEFAULT_SERIAL_LOG_LINES", 10000))
        no_stub_read = bool(self.no_stub_read_var.get())
        no_stub_write = bool(self.no_stub_write_var.get())
        self.app.settings.update({
            "backup_folder": self.backup_folder_var.get().strip() or settings_mod.DEFAULT_BACKUP_FOLDER,
            "download_url": self.download_url_var.get().strip() or settings_mod.DEFAULT_DOWNLOAD_URL,
            "serial_log_buffer_lines": n,
            "baud_rate_read": self.baud_read_var.get(),
            "baud_rate_write": self.baud_write_var.get(),
            "no_stub_read": no_stub_read,
            "no_stub_write": no_stub_write,
        })
        set_no_stub_read(no_stub_read)
        set_no_stub_write(no_stub_write)
        settings_mod.save(self.app.settings)


class App(ctk.CTk):
    def __init__(self):
        super().__init__()
        self.title("rdzTTGOsonde Flasher")
        self.geometry("720x520")
        self.minsize(500, 400)
        _icon_path = os.path.join(os.path.dirname(__file__), "rdzttgosonde.png")
        if os.path.isfile(_icon_path):
            try:
                self.iconphoto(True, tk.PhotoImage(file=os.path.abspath(_icon_path)))
            except Exception:
                pass

        self.settings = settings_mod.load()
        set_no_stub_read(bool(self.settings.get("no_stub_read", True)))
        set_no_stub_write(bool(self.settings.get("no_stub_write", False)))

        self.wifi_host_var = tk.StringVar(value=self.settings.get("wifi_host", "rdzsonde.local"))
        self.wifi_user_var = tk.StringVar(value=self.settings.get("wifi_user", ""))
        self.wifi_pass_var = tk.StringVar(value=self.settings.get("wifi_pass", ""))
        # Default "Save password" to True if config already has a password (backward compat)
        self.wifi_save_pass_var = ctk.BooleanVar(
            value=self.settings.get("wifi_save_password", bool(self.settings.get("wifi_pass", "")))
        )
        self.port_var = tk.StringVar(value="Auto")

        self.grid_columnconfigure(0, weight=1)
        self.grid_rowconfigure(1, weight=1)

        # Top bar: segmented control – always start on Flash tab
        last_mode = "Flash"
        self.mode_var = ctk.StringVar(value=last_mode)
        self.seg = ctk.CTkSegmentedButton(
            self,
            values=MODE_VALUES,
            variable=self.mode_var,
            command=self._on_mode_change,
            dynamic_resizing=False,
        )
        self.seg.grid(row=0, column=0, sticky="ew", padx=15, pady=(15, 10))

        # Content area (single cell; we swap frames)
        self.content_container = ctk.CTkFrame(self, fg_color="transparent")
        self.content_container.grid(row=1, column=0, sticky="nsew", padx=15, pady=(0, 15))
        self.content_container.grid_columnconfigure(0, weight=1)
        self.content_container.grid_rowconfigure(0, weight=1)

        self.frames: Dict[str, ContentFrame] = {}
        for Cls, key in [
            (FlashFrame, "Flash"),
            (SerialLogFrame, "Serial"),
            (USBManageFrame, "USB"),
            (FilesWiFiFrame, "Wi-Fi"),
            (SDWiFiFrame, "SD (via WiFi)"),
            (SettingsFrame, "Settings"),
        ]:
            f = Cls(self.content_container, app=self)
            f.grid(row=0, column=0, sticky="nsew")
            f.hide()
            self.frames[key] = f

        self.frames["Settings"].load_from_settings(self.settings)
        ports = get_serial_ports()
        lp = (self.settings.get("last_port") or "").strip()
        if lp and lp in ports:
            self.port_var.set(lp)
        else:
            self.port_var.set(ports[0] if ports else "Auto")

        self._on_mode_change(last_mode)
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    def _on_close(self) -> None:
        self.settings["wifi_host"] = self.wifi_host_var.get()
        self.settings["wifi_user"] = self.wifi_user_var.get()
        self.settings["wifi_save_password"] = self.wifi_save_pass_var.get()
        if self.wifi_save_pass_var.get():
            self.settings["wifi_pass"] = self.wifi_pass_var.get()
        else:
            self.settings.pop("wifi_pass", None)
        self.settings["last_port"] = self.port_var.get()
        if "Serial" in self.frames and hasattr(self.frames["Serial"], "baud_combo"):
            self.settings["baud_rate"] = self.frames["Serial"].baud_combo.get()
        settings_mod.save(self.settings)
        self.destroy()

    def _on_mode_change(self, value: str) -> None:
        if "Settings" in self.frames and self.mode_var.get() != "Settings":
            sf = self.frames["Settings"]
            if hasattr(sf, "_save_from_ui"):
                sf._save_from_ui()
        for k, frame in self.frames.items():
            if k == value:
                frame.show()
            else:
                frame.hide()
        settings_mod.save(self.settings)


def main():
    import os
    try:
        import esptool  # noqa: F401
    except ModuleNotFoundError:
        import tkinter as _tk
        _tk.messagebox.showerror(
            "Missing dependency",
            "esptool not found. Please run:\n  pip install -r requirements.txt\n"
            "(Use the same Python/venv you use to run this app, e.g. from ttgoconfig folder or project root.)"
        )
        return
    ctk.set_appearance_mode("system")
    theme_path = os.path.join(os.path.dirname(__file__), "theme_fresh_blue.json")
    if os.path.isfile(theme_path):
        ctk.set_default_color_theme(theme_path)
    else:
        ctk.set_default_color_theme("blue")
    app = App()
    app.mainloop()


if __name__ == "__main__":
    main()

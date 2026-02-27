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
from tkinter import filedialog, messagebox
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
    BLOCK_SIZE as LFS_BLOCK_SIZE,
)
from .firmware import (
    fetch_manifest_firmware_url as _fetch_firmware_url_for_source,
    fetch_download_html_versions as _fetch_download_html_versions,
    download_firmware_to_temp,
)
from . import wifi_ops as wifi_ops_mod
from . import sd_ops as sd_ops_mod

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
        if current in ports:
            self.port_combo.set(current)

    def _on_line_cb(self) -> Callable[[str], None]:
        """Return a thread-safe on_line callback that appends to this frame's log."""
        return lambda line: self.after(0, lambda l=line: self._log(l))

    def _make_port_combo_row(self, parent: Any) -> int:
        """Add Port label, port_combo, and Refresh button to parent at row 0. Returns next available column index."""
        parent.grid_columnconfigure(1, weight=1)
        ctk.CTkLabel(parent, text="Port:").grid(row=0, column=0, padx=(0, 10), pady=5)
        self.port_combo = ctk.CTkComboBox(parent, values=get_serial_ports(), width=220)
        self.port_combo.grid(row=0, column=1, padx=0, pady=5, sticky="w")
        ctk.CTkButton(parent, text="Refresh", width=80, command=self._refresh_ports).grid(
            row=0, column=2, padx=(10, 0), pady=5
        )
        return 3


def _run_in_background(
    work: Callable[..., tuple[bool, str]],
    on_done: Callable[[bool, str], None],
    on_progress: Optional[Callable[[int, int], None]] = None,
) -> None:
    """Run work() in a thread; on_done(ok, msg) on main thread. If on_progress is set, work(progress_cb) receives a callback progress_cb(n, total)."""
    def run():
        try:
            if on_progress is not None:
                root = tk._default_root
                def progress_cb(n: int, total: int) -> None:
                    if root and on_progress:
                        root.after(0, lambda: on_progress(n, total))
                ok, msg = work(progress_cb)
            else:
                ok, msg = work()
        except Exception as e:
            ok, msg = False, str(e)
        if on_done:
            root = tk._default_root
            if root:
                root.after(0, lambda: on_done(ok, msg))
            else:
                on_done(ok, msg)
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
            self.backup_combo = ctk.CTkComboBox(self.fw_opts_frame, values=backups, width=280)
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
        path = filedialog.askopenfilename(title="Select firmware image", filetypes=[("Binary", "*.bin"), ("All", "*.*")])
        if path:
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

            _run_in_background(work, on_done)
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

        _run_in_background(work, on_done)


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
        ctk.CTkButton(actions_frame, text="Setup WiFi via IMPROV").grid(
            row=4, column=1, sticky="ew", padx=(10, 0), pady=3)

        row += 1
        self._op_log = self._make_log_widget(height=100, state="disabled")
        self._op_log.grid(row=row, column=0, sticky="nsew", padx=20, pady=(10, 20))
        self.grid_rowconfigure(row, weight=1)

    def _make_backup(self) -> None:
        folder = settings_mod.get_backup_folder_expanded(self.app.settings if self.app else {})
        os.makedirs(folder, exist_ok=True)
        from datetime import datetime
        default_name = "backup-%s.bin" % datetime.now().strftime("%Y%m%d-%H%M")
        path = filedialog.asksaveasfilename(
            title="Save backup as", initialdir=folder, initialfile=default_name,
            defaultextension=".bin", filetypes=[("Binary", "*.bin"), ("All", "*.*")]
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

        _run_in_background(work, on_done)

    def _restore_full_backup(self) -> None:
        path = filedialog.askopenfilename(
            title="Select backup image", filetypes=[("Binary", "*.bin"), ("All", "*.*")]
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

        _run_in_background(work, on_done)

    def _restore_selected_files(self) -> None:
        messagebox.showinfo("Restore selected files", "Not yet implemented.")

    def _downloadfs(self) -> None:
        dest_dir = filedialog.askdirectory(title="Select directory to extract filesystem into")
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
                self._log("Filesystem extracted to %s\n" % dest_dir)
                messagebox.showinfo("Extract filesystem", "Filesystem extracted to:\n%s" % dest_dir)
            else:
                messagebox.showerror("Extract filesystem", "Failed. Check the log.")

        _run_in_background(work, on_done)

    def _uploadfs(self) -> None:
        src_dir = filedialog.askdirectory(title="Select directory to upload as filesystem")
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
                self._log("Filesystem uploaded successfully.\n")
                messagebox.showinfo("Upload filesystem", "Filesystem uploaded successfully.")
            else:
                messagebox.showerror("Upload filesystem", "Failed. Check the log.")

        _run_in_background(work, on_done)

    def _extractfs_from_backup(self) -> None:
        backup_path = filedialog.askopenfilename(
            title="Select backup image (.bin)",
            filetypes=[("Binary", "*.bin"), ("All", "*.*")],
        )
        if not backup_path:
            return
        dest_dir = filedialog.askdirectory(title="Select directory to extract filesystem into")
        if not dest_dir:
            return
        self._log("\n--- Extract filesystem from backup %s to %s ---\n" % (backup_path, dest_dir))

        def work():
            try:
                lfs_extract_from_backup(backup_path, dest_dir, PARTITION_SPIFFS_OFFSET, PARTITION_SPIFFS_SIZE)
                return True, ""
            except Exception as e:
                return False, str(e)

        def on_done(ok: bool, msg: str) -> None:
            if ok:
                self._log("Filesystem extracted to %s\n" % dest_dir)
                messagebox.showinfo("Extract filesystem", "Filesystem extracted to:\n%s" % dest_dir)
            else:
                self._log("Error: %s\n" % msg)
                messagebox.showerror("Extract filesystem", "Failed: %s" % msg)

        _run_in_background(work, on_done)


class WiFiManageFrame(ContentFrame):
    def __init__(self, master, app: Optional["App"] = None, **kwargs):
        super().__init__(master, "Wi-Fi", app=app, **kwargs)
        self.grid_columnconfigure(0, weight=1)

        row = 0
        host_frame = ctk.CTkFrame(self, fg_color="transparent")
        host_frame.grid(row=row, column=0, sticky="ew", padx=20, pady=(10, 4))
        host_frame.grid_columnconfigure(1, weight=1)
        ctk.CTkLabel(host_frame, text="Host:").grid(row=0, column=0, sticky="w", padx=(0, 8), pady=0)
        self.host_entry = ctk.CTkEntry(
            host_frame,
            textvariable=self.app.wifi_host_var,
            placeholder_text="rdzsonde.local or IP",
        )
        self.host_entry.grid(row=0, column=1, sticky="ew", padx=(0, 10), pady=0)
        self.ip_label = ctk.CTkLabel(host_frame, text="", text_color="gray", width=160, anchor="w")
        self.ip_label.grid(row=0, column=2, sticky="w", padx=(0, 10), pady=0)
        ctk.CTkButton(host_frame, text="Resolve", width=80, command=self._resolve_host).grid(row=0, column=3, padx=0, pady=0)
        ctk.CTkButton(host_frame, text="Test connection", width=120, command=self._test_connection).grid(
            row=0, column=4, padx=(10, 0), pady=0
        )
        row += 1

        auth_frame = ctk.CTkFrame(self, fg_color="transparent")
        auth_frame.grid(row=row, column=0, sticky="ew", padx=20, pady=(4, 4))
        auth_frame.grid_columnconfigure(1, weight=1)
        ctk.CTkLabel(auth_frame, text="Auth (optional):").grid(row=0, column=0, sticky="w", padx=(0, 8), pady=0)
        ctk.CTkLabel(auth_frame, text="User").grid(row=0, column=2, padx=(0, 4), pady=0)
        self.auth_user_entry = ctk.CTkEntry(
            auth_frame,
            width=120,
            textvariable=self.app.wifi_user_var,
            placeholder_text="username",
        )
        self.auth_user_entry.grid(row=0, column=3, padx=(0, 12), pady=0, sticky="w")
        ctk.CTkLabel(auth_frame, text="Password").grid(row=0, column=4, padx=(0, 4), pady=0)
        self.auth_pass_entry = ctk.CTkEntry(
            auth_frame,
            width=120,
            textvariable=self.app.wifi_pass_var,
            placeholder_text="password",
            show="•",
        )
        self.auth_pass_entry.grid(row=0, column=5, padx=0, pady=0, sticky="w")
        row += 1

        ctk.CTkLabel(self, text="Actions:", font=ctk.CTkFont(weight="bold")).grid(
            row=row, column=0, sticky="w", padx=20, pady=(8, 4))
        row += 1
        for a in ["Restore selected files from backup", "Restore from backup .bin to TTGO"]:
            ctk.CTkButton(self, text=a).grid(
                row=row, column=0, sticky="ew", padx=20, pady=3)
            row += 1
        ctk.CTkButton(
            self, text="Download files from TTGO",
            command=self._extract_files,
        ).grid(row=row, column=0, sticky="ew", padx=20, pady=3)
        row += 1

        self.grid_rowconfigure(row, weight=1)
        self.wifi_log = self._make_log_widget(height=160, wrap="word", state="disabled")
        self.wifi_log.grid(row=row, column=0, sticky="nsew", padx=20, pady=(8, 10))

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
            self._log("Resolved %s → %s\n" % (host, ip))
        except Exception as e:
            self._resolved_ip = ""
            if hasattr(self, "ip_label"):
                self.ip_label.configure(text="")
            self._log("Resolve failed: %s\n" % e)

    def _test_connection(self) -> None:
        host = (self.host_entry.get() or "").strip() or "rdzsonde.local"
        self._log("\n--- Test connection: %s ---\n" % host)

        def work():
            return wifi_ops_mod.test_connection(host)

        def on_done(ok: bool, msg: str) -> None:
            if ok:
                self._log("Connected.\n%s\n" % (msg[:300] + "..." if len(msg) > 300 else msg))
            else:
                self._log("Failed: %s\n" % msg)

        _run_in_background(work, on_done)

    def _extract_files(self) -> None:
        folder = filedialog.askdirectory(title="Choose folder to save files")
        if not folder:
            return
        host = (self.host_entry.get() or "").strip() or "rdzsonde.local"
        user = (self.auth_user_entry.get() or "").strip()
        password = (self.auth_pass_entry.get() or "").strip()
        session, base_url, err = wifi_ops_mod.session_for_host(host, user or None, password or None)
        if err:
            self._log("Error: %s\n" % err)
            return
        files_to_get = wifi_ops_mod.FILE_SETS["all"]
        self._log("\n--- Download files from %s → %s ---\n" % (host, folder))

        def work():
            saved = 0
            for name in files_to_get:
                content, ferr = wifi_ops_mod.get_file(session, base_url, name)
                if ferr:
                    if "404" in ferr:
                        self.after(0, lambda n=name: self._log("  skip %s (not found)\n" % n))
                        continue
                    return False, ferr
                path = os.path.join(folder, name)
                with open(path, "wb") as f:
                    f.write(content)
                saved += 1
                self.after(0, lambda n=name: self._log("  saved %s\n" % n))
            return True, "Done – %d file(s) downloaded to %s\n" % (saved, folder)

        def on_done(ok: bool, msg: str) -> None:
            self._log(msg if msg.endswith("\n") else msg + "\n")

        _run_in_background(work, on_done)

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
                self.sd_up_btn.configure(state="normal" if self._sd_current_dir else "disabled")
            else:
                self.sd_placeholder.configure(text="Error: %s" % (msg if not ok else "Unknown"))
                self.sd_placeholder.grid()
                self.sd_scroll.grid_remove()
        _run_in_background(work, on_done)

    def _sd_populate_list(self) -> None:
        for child in self.sd_scroll.winfo_children():
            child.destroy()
        self._sd_row_vars.clear()

        def _format_ts(ts: str) -> str:
            if not ts:
                return ""
            try:
                dt = datetime.datetime.fromisoformat(ts.replace("Z", "+00:00"))
            except Exception:
                return ts
            return dt.strftime("%Y-%m-%d %H:%M")

        entries = sorted(
            self._sd_entries,
            key=lambda it: (
                0 if it.get("dir") else 1,
                str(it.get("name", "")).lower(),
            ),
        )

        for i, item in enumerate(entries):
            name = item.get("name", "?")
            is_dir = bool(item.get("dir"))
            size = int(item.get("size", 0)) if not is_dir else 0
            ts = item.get("ts", "")

            row = ctk.CTkFrame(self.sd_scroll, fg_color="transparent")
            row.grid(row=i, column=0, sticky="ew")
            row.grid_columnconfigure(1, weight=1)

            var = ctk.BooleanVar(value=False)
            ctk.CTkCheckBox(row, text="", variable=var, width=18).grid(
                row=0, column=0, padx=(0, 4), pady=0
            )

            prefix = "▸" if is_dir else "•"
            display_name = name + ("/" if is_dir else "")
            name_lbl = ctk.CTkLabel(row, text=f"{prefix} {display_name}", anchor="w")
            name_lbl.grid(row=0, column=1, sticky="w", padx=0, pady=0)

            size_text = "" if is_dir else f"{size} B"
            size_lbl = ctk.CTkLabel(row, text=size_text, text_color="gray")
            size_lbl.grid(row=0, column=2, padx=(8, 0), sticky="e")

            ts_text = _format_ts(ts)
            ts_lbl = ctk.CTkLabel(row, text=ts_text, text_color="gray")
            ts_lbl.grid(row=0, column=3, padx=(8, 0), sticky="e")

            if is_dir:
                def enter_dir(event=None, d=self._sd_current_dir, n=name):
                    self._sd_current_dir = (d + "/" + n).strip("/") if d else n
                    self._sd_refresh()
                row.bind("<Double-Button-1>", enter_dir)
                name_lbl.bind("<Double-Button-1>", enter_dir)

            self._sd_row_vars.append((var, item))

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
        dest = filedialog.askdirectory(title="Choose folder to save SD files")
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
                messagebox.showinfo("SD Download", msg)
            else:
                messagebox.showerror("SD Download", msg)

        self.sd_progress.set(0)
        self.sd_progress_label.configure(text="Starting…")
        _run_in_background(work, on_done, on_progress)

    def _sd_download_selected(self) -> None:
        selected = [(var.get(), entry) for var, entry in self._sd_row_vars if var.get()]
        if not selected:
            messagebox.showwarning("SD Download", "Select one or more items (files or folders) to download.")
            return
        dest = filedialog.askdirectory(title="Choose folder to save SD files")
        if not dest:
            return
        try:
            session, base_url = self._sd_get_session_and_base()
        except RuntimeError as e:
            messagebox.showerror("SD Download", str(e))
            return

        def work(progress_cb=None):
            all_paths: list[str] = []
            for _checked, entry in selected:
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
                messagebox.showinfo("SD Download", msg)
            else:
                messagebox.showerror("SD Download", msg)

        self.sd_progress.set(0)
        self.sd_progress_label.configure(text="Starting…")
        _run_in_background(work, on_done, on_progress)


class SDWiFiFrame(WiFiManageFrame):
    """SD card download via Wi-Fi in its own tab."""

    def __init__(self, master, app: Optional["App"] = None, **kwargs):
        ContentFrame.__init__(self, master, "SD (via WiFi)", app=app, **kwargs)
        self.grid_columnconfigure(0, weight=1)

        row = 0
        host_frame = ctk.CTkFrame(self, fg_color="transparent")
        host_frame.grid(row=row, column=0, sticky="ew", padx=20, pady=(10, 4))
        host_frame.grid_columnconfigure(1, weight=1)
        ctk.CTkLabel(host_frame, text="Host:").grid(row=0, column=0, sticky="w", padx=(0, 8), pady=0)
        self.host_entry = ctk.CTkEntry(
            host_frame,
            textvariable=self.app.wifi_host_var,
            placeholder_text="rdzsonde.local or IP",
        )
        self.host_entry.grid(row=0, column=1, sticky="ew", padx=(0, 10), pady=0)
        self.ip_label = ctk.CTkLabel(host_frame, text="", text_color="gray", width=160, anchor="w")
        self.ip_label.grid(row=0, column=2, sticky="w", padx=(0, 10), pady=0)
        ctk.CTkButton(host_frame, text="Resolve", width=80, command=self._resolve_host).grid(row=0, column=3, padx=0, pady=0)
        ctk.CTkButton(host_frame, text="Test connection", width=120, command=self._test_connection).grid(
            row=0, column=4, padx=(10, 0), pady=0
        )
        row += 1

        auth_frame = ctk.CTkFrame(self, fg_color="transparent")
        auth_frame.grid(row=row, column=0, sticky="ew", padx=20, pady=(4, 4))
        auth_frame.grid_columnconfigure(1, weight=1)
        ctk.CTkLabel(auth_frame, text="Auth (optional):").grid(row=0, column=0, sticky="w", padx=(0, 8), pady=0)
        ctk.CTkLabel(auth_frame, text="User").grid(row=0, column=2, padx=(0, 4), pady=0)
        self.auth_user_entry = ctk.CTkEntry(
            auth_frame,
            width=120,
            textvariable=self.app.wifi_user_var,
            placeholder_text="username",
        )
        self.auth_user_entry.grid(row=0, column=3, padx=(0, 12), pady=0, sticky="w")
        ctk.CTkLabel(auth_frame, text="Password").grid(row=0, column=4, padx=(0, 4), pady=0)
        self.auth_pass_entry = ctk.CTkEntry(
            auth_frame,
            width=120,
            textvariable=self.app.wifi_pass_var,
            placeholder_text="password",
            show="•",
        )
        self.auth_pass_entry.grid(row=0, column=5, padx=0, pady=0, sticky="w")
        row += 1

        # SD card download: browse folders (device /files.json style), then download all or selected folder(s)
        ctk.CTkLabel(self, text="SD card download", font=ctk.CTkFont(weight="bold")).grid(
            row=row, column=0, sticky="w", padx=20, pady=(18, 6))
        row += 1
        sd_desc = ctk.CTkLabel(
            self, text="Browse folders (like device list directory). Select items with checkboxes, then download all or selected.",
            text_color="gray"
        )
        sd_desc.grid(row=row, column=0, sticky="w", padx=20, pady=(0, 4))
        row += 1
        sd_frame = ctk.CTkFrame(self, fg_color="transparent")
        sd_frame.grid(row=row, column=0, sticky="nsew", padx=20, pady=4)
        sd_frame.grid_columnconfigure(0, weight=1)
        self.grid_rowconfigure(row, weight=1)
        self._sd_entries = []
        self._sd_row_vars = []
        self.sd_placeholder = ctk.CTkLabel(
            sd_frame, text="Click Refresh list to load SD root (requires device connected via Wi-Fi).",
            text_color="gray"
        )
        self.sd_placeholder.grid(row=0, column=0, sticky="nsew", pady=(0, 6))
        self.sd_scroll = ctk.CTkScrollableFrame(sd_frame, height=120, fg_color=("gray90", "gray20"))
        self.sd_scroll.grid(row=0, column=0, columnspan=2, sticky="nsew", pady=(0, 6))
        self.sd_scroll.grid_remove()
        sd_frame.grid_rowconfigure(0, weight=1)
        self._setup_sd_scroll_binding()
        sd_btns = ctk.CTkFrame(sd_frame, fg_color="transparent")
        sd_btns.grid(row=1, column=0, columnspan=2, sticky="w", pady=(0, 4))
        self.sd_refresh_btn = ctk.CTkButton(sd_btns, text="Refresh list", width=100, command=self._sd_refresh)
        self.sd_refresh_btn.grid(row=0, column=0, padx=(0, 10), pady=0)
        self.sd_up_btn = ctk.CTkButton(sd_btns, text="Up", width=50, command=self._sd_go_up, state="disabled")
        self.sd_up_btn.grid(row=0, column=1, padx=0, pady=0)
        ctk.CTkButton(sd_btns, text="Download all", width=120, command=self._sd_download_all).grid(row=0, column=2, padx=(10, 0), pady=0)
        ctk.CTkButton(sd_btns, text="Download selected", width=140, command=self._sd_download_selected).grid(row=0, column=3, padx=(10, 0), pady=0)
        self._sd_current_dir = ""
        self.sd_progress = ctk.CTkProgressBar(sd_frame, height=8, progress_color=("gray70", "gray35"))
        self.sd_progress.grid(row=2, column=0, columnspan=2, sticky="ew", pady=(4, 0))
        self.sd_progress.set(0)
        self.sd_progress_label = ctk.CTkLabel(sd_frame, text="", text_color="gray", height=0)
        self.sd_progress_label.grid(row=3, column=0, columnspan=2, sticky="w", pady=(2, 0))

    def _setup_sd_scroll_binding(self) -> None:
        """Register a single bind_all handler that scrolls sd_scroll whenever the mouse
        pointer is physically over the scroll area, regardless of which widget received
        the event (works on macOS where scroll events may go to the focused widget)."""
        import sys

        def _over_sd() -> bool:
            pf = self.sd_scroll._parent_frame
            try:
                mx, my = pf.winfo_pointerx(), pf.winfo_pointery()
                x1, y1 = pf.winfo_rootx(), pf.winfo_rooty()
                x2, y2 = x1 + pf.winfo_width(), y1 + pf.winfo_height()
                return x1 <= mx <= x2 and y1 <= my <= y2
            except Exception:
                return False

        def _on_wheel(event):
            if not _over_sd():
                return
            canvas = self.sd_scroll._parent_canvas
            if sys.platform.startswith("win"):
                canvas.yview_scroll(int(-event.delta / 6), "units")
            else:
                canvas.yview_scroll(-event.delta, "units")
            return "break"

        def _on_btn4(event):
            if _over_sd():
                self.sd_scroll._parent_canvas.yview_scroll(-1, "units")

        def _on_btn5(event):
            if _over_sd():
                self.sd_scroll._parent_canvas.yview_scroll(1, "units")

        # Use tkinter.Misc.bind_all directly — CTkBaseClass overrides bind_all and
        # raises AttributeError to prevent accidental global bindings, so we bypass it.
        import tkinter
        tkinter.Misc.bind_all(self, "<MouseWheel>", _on_wheel, add="+")
        tkinter.Misc.bind_all(self, "<Button-4>", _on_btn4, add="+")
        tkinter.Misc.bind_all(self, "<Button-5>", _on_btn5, add="+")

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

        _run_in_background(work, on_done)



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
        ctk.CTkButton(bot, text="Save to file", width=100, command=self._save_log).grid(row=0, column=0, padx=(0, 10), pady=0)
        self.strip_ansi_var = ctk.BooleanVar(value=False)
        ctk.CTkCheckBox(bot, text="Strip ANSI when saving", variable=self.strip_ansi_var,
                        command=self._save_strip_ansi_setting).grid(
            row=0, column=1, padx=10, pady=0
        )
        ctk.CTkButton(bot, text="Clear", width=80, command=self._clear_log).grid(row=0, column=2, padx=10, pady=0)
        ctk.CTkButton(bot, text="Copy", width=80, command=self._copy_log).grid(row=0, column=3, padx=0, pady=0)

    def _save_strip_ansi_setting(self) -> None:
        if self.app:
            self.app.settings["strip_ansi_when_saving"] = bool(self.strip_ansi_var.get())
            settings_mod.save(self.app.settings)

    def _append_log(self, text: str) -> None:
        """Append text to serial log with ANSI color parsing."""
        segments = _parse_ansi_to_segments(text)
        tb = self._serial_log_tk or self.log_text

        def do():
            self.log_text.configure(state="normal")
            for seg_text, tag in segments:
                if tag:
                    tb.insert("end", seg_text, tag)
                else:
                    tb.insert("end", seg_text)
            self.log_text.see("end")
            self.log_text.configure(state="disabled")
        self.after(0, do)

    def _strip_ansi(self, s: str) -> str:
        import re
        return re.sub(r"\x1b\[[0-9;]*m", "", s)

    def _save_log(self) -> None:
        path = filedialog.asksaveasfilename(
            title="Save log", defaultextension=".txt", filetypes=[("Text", "*.txt"), ("All", "*.*")]
        )
        if not path:
            return
        try:
            content = self.log_text.get("1.0", "end")
            if self.strip_ansi_var.get():
                content = self._strip_ansi(content)
            with open(path, "w", encoding="utf-8", errors="replace") as f:
                f.write(content)
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
            self._serial_queue.put("\n[Error: %s]\n" % e)
        finally:
            self._serial_queue.put("\n[Disconnected]\n")

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
                self.log_text.configure(state="normal")
                for seg_text, tag in segments:
                    if tag:
                        tb.insert("end", seg_text, tag)
                    else:
                        tb.insert("end", seg_text)
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
        if self.app and self.app.settings.get("strip_ansi_when_saving") is not None:
            self.strip_ansi_var.set(bool(self.app.settings["strip_ansi_when_saving"]))

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

        self.baud_read_var = ctk.StringVar(value="115200")
        self.no_stub_read_var = ctk.BooleanVar(value=False)
        ctk.CTkLabel(self, text="read-flash baud rate:", width=LABEL_W, anchor="w").grid(
            row=row, column=0, sticky="w", padx=(20, 10), pady=4)
        ctk.CTkComboBox(self, values=["115200", "460800", "921600"], variable=self.baud_read_var, width=120).grid(
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
        d = filedialog.askdirectory(title="Select backup folder")
        if d:
            self.backup_folder_var.set(d)
            self._save_from_ui()

    def load_from_settings(self, s: Dict[str, Any]) -> None:
        self.backup_folder_var.set(s.get("backup_folder", settings_mod.DEFAULT_BACKUP_FOLDER))
        self.download_url_var.set(s.get("download_url", settings_mod.DEFAULT_DOWNLOAD_URL))
        self.serial_log_lines_var.set(str(s.get("serial_log_buffer_lines", settings_mod.DEFAULT_SERIAL_LOG_LINES)))
        self.baud_read_var.set(s.get("baud_rate_read", settings_mod.DEFAULT_BAUD_READ))
        self.baud_write_var.set(s.get("baud_rate_write", settings_mod.DEFAULT_BAUD_WRITE))
        self.no_stub_read_var.set(bool(s.get("no_stub_read", False)))
        self.no_stub_write_var.set(bool(s.get("no_stub_write", False)))

    def _save_from_ui(self) -> None:
        if not self.app:
            return
        try:
            n = int(self.serial_log_lines_var.get().strip() or "10000")
        except ValueError:
            n = settings_mod.DEFAULT_SERIAL_LOG_LINES
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

        self.settings = settings_mod.load()
        set_no_stub_read(bool(self.settings.get("no_stub_read", False)))
        set_no_stub_write(bool(self.settings.get("no_stub_write", False)))

        self.wifi_host_var = tk.StringVar(value="rdzsonde.local")
        self.wifi_user_var = tk.StringVar(value="")
        self.wifi_pass_var = tk.StringVar(value="")

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
            (WiFiManageFrame, "Wi-Fi"),
            (SDWiFiFrame, "SD (via WiFi)"),
            (SettingsFrame, "Settings"),
        ]:
            f = Cls(self.content_container, app=self)
            f.grid(row=0, column=0, sticky="nsew")
            f.hide()
            self.frames[key] = f

        self.frames["Settings"].load_from_settings(self.settings)
        if self.settings.get("last_port"):
            lp = self.settings["last_port"]
            for k in ("Flash", "USB", "Serial"):
                if k in self.frames and hasattr(self.frames[k], "port_combo"):
                    combo = self.frames[k].port_combo
                    vals = list(combo.cget("values"))
                    if lp in vals:
                        combo.set(lp)

        self._on_mode_change(last_mode)
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    def _on_close(self) -> None:
        self.settings["last_mode"] = self.mode_var.get()
        for k in ("Flash", "USB", "Serial"):
            if k in self.frames and hasattr(self.frames[k], "port_combo"):
                self.settings["last_port"] = self.frames[k].port_combo.get()
                break
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
        self.settings["last_mode"] = value
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

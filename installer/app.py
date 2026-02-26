"""
rdzTTGOsonde Desktop Flasher – UI implementation per plan.

Top-level: Flash | USB | Wi-Fi | Serial | Settings.
Settings persisted; Flash/USB use esptool; Serial uses pyserial; Wi-Fi uses HTTP.

Run: python -m installer.app
"""

import os
import queue
import re
import threading
import tkinter as tk
from html.parser import HTMLParser
from tkinter import filedialog, messagebox
from typing import Any, Callable, Dict, List, Optional, Tuple

import customtkinter as ctk

from . import settings as settings_mod
from .esptool_helper import flash_firmware, read_backup, write_backup

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


def _fetch_firmware_url_for_source(
    base_url: str, choice: str
) -> tuple[Optional[str], Optional[str]]:
    """Return (firmware_url, error_msg). choice is 'Stable (main)' or 'Development (dev2)'."""
    base = base_url.rstrip("/")
    manifest_url = base + "/manifest.json"
    try:
        import urllib.request
        req = urllib.request.Request(manifest_url)
        with urllib.request.urlopen(req, timeout=15) as r:
            import json as _json
            data = _json.loads(r.read().decode("utf-8"))
    except Exception as e:
        return None, "Failed to fetch manifest: %s" % e
    builds = data.get("builds") or []
    want_main = "main" in choice.lower() or "Stable" in choice
    for b in builds:
        fw = (b.get("fwversion") or "").strip()
        if want_main and fw.startswith("main"):
            parts = b.get("parts") or []
            if not parts:
                return None, "Manifest build has no parts."
            path = (parts[0].get("path") or "").strip()
            if not path:
                return None, "Manifest part has no path."
            return base + "/" + path.lstrip("/"), None
        if not want_main and (fw.startswith("dev") or "dev" in choice.lower()):
            parts = b.get("parts") or []
            if not parts:
                return None, "Manifest build has no parts."
            path = (parts[0].get("path") or "").strip()
            if not path:
                return None, "Manifest part has no path."
            return base + "/" + path.lstrip("/"), None
    return None, "No matching build in manifest (Stable/main or Development/dev2)."


class _DownloadTableParser(HTMLParser):
    """Extract (version, href) from download.html data-table rows that contain a -full.bin link."""
    def __init__(self) -> None:
        super().__init__()
        self.rows: List[Tuple[str, str]] = []
        self._in_table = False
        self._in_row = False
        self._current_code = ""
        self._current_href = ""
        self._in_code = False
        self._in_a = False

    def handle_starttag(self, tag: str, attrs: list[tuple[str, str | None]]) -> None:
        if tag == "table" and any(k == "class" and (v or "").find("data-table") != -1 for k, v in attrs):
            self._in_table = True
        if self._in_table and tag == "tr":
            self._in_row = True
            self._current_code = ""
            self._current_href = ""
        if self._in_row and tag == "code":
            self._in_code = True
        if self._in_row and tag == "a":
            self._in_a = True
            for k, v in attrs:
                if k == "href" and v and v.endswith("-full.bin"):
                    self._current_href = v
                    break

    def handle_endtag(self, tag: str) -> None:
        if tag == "table" and self._in_table:
            self._in_table = False
        if tag == "tr" and self._in_row:
            if self._current_code and self._current_href:
                self.rows.append((self._current_code.strip(), self._current_href))
            self._in_row = False
        if tag == "code":
            self._in_code = False
        if tag == "a":
            self._in_a = False

    def handle_data(self, data: str) -> None:
        if self._in_code:
            self._current_code += data


def _fetch_download_html_versions(base_url: str) -> Tuple[List[Tuple[str, str]], Optional[str]]:
    """Fetch download.html from base_url and parse for (version, href) full-image rows. Return ([(display, url)], error)."""
    url = base_url.rstrip("/") + "/download.html"
    try:
        import urllib.request
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

    def show(self) -> None:
        self.grid(row=0, column=0, sticky="nsew")

    def hide(self) -> None:
        self.grid_remove()


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
        if on_done and tk._default_root:
            tk._default_root.after(0, lambda: on_done(ok, msg))
    t = threading.Thread(target=run, daemon=True)
    t.start()


class FlashFrame(ContentFrame):
    def __init__(self, master, app: Optional["App"] = None, **kwargs):
        super().__init__(master, "Flash", app=app, **kwargs)
        self.grid_columnconfigure(0, weight=1)

        row = 0
        dev_frame = ctk.CTkFrame(self, fg_color="transparent")
        dev_frame.grid(row=row, column=0, sticky="ew", padx=20, pady=(20, 10))
        dev_frame.grid_columnconfigure(1, weight=1)
        ctk.CTkLabel(dev_frame, text="Port:").grid(row=0, column=0, padx=(0, 10), pady=5)
        self.port_combo = ctk.CTkComboBox(dev_frame, values=get_serial_ports(), width=220)
        self.port_combo.grid(row=0, column=1, padx=0, pady=5, sticky="w")
        ref_btn = ctk.CTkButton(dev_frame, text="Refresh", width=80, command=self._refresh_ports)
        ref_btn.grid(row=0, column=2, padx=(10, 0), pady=5)
        row += 1

        src_label = ctk.CTkLabel(self, text="Firmware source:")
        src_label.grid(row=row, column=0, sticky="w", padx=20, pady=(10, 5))
        row += 1
        self.fw_source = ctk.CTkSegmentedButton(
            self, values=["Download from website", "Local file", "From backup"], command=self._on_fw_source_change
        )
        self.fw_source.set("Download from website")
        self.fw_source.grid(row=row, column=0, sticky="ew", padx=20, pady=5)
        row += 1

        self.fw_opts_frame = ctk.CTkFrame(self, fg_color="transparent")
        self.fw_opts_frame.grid(row=row, column=0, sticky="ew", padx=20, pady=10)
        self.fw_opts_frame.grid_columnconfigure(0, weight=1)
        self._build_fw_opts()
        row += 1

        self.flash_btn = ctk.CTkButton(
            self, text="Flash", height=40, font=ctk.CTkFont(size=14, weight="bold"), command=self._do_flash
        )
        self.flash_btn.grid(row=row, column=0, padx=20, pady=20, sticky="ew")
        row += 1

        self.log_text = ctk.CTkTextbox(self, height=120, state="disabled")
        self.log_text.grid(row=row, column=0, sticky="nsew", padx=20, pady=(0, 20))
        self.grid_rowconfigure(row, weight=1)
        _configure_ansi_tags_for_log(self.log_text)
        self._flash_running = False

    def _refresh_ports(self) -> None:
        self.port_combo.configure(values=get_serial_ports())

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
            ok, msg, rows = work()
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

    def _log(self, text: str) -> None:
        actions = _parse_ansi_to_actions_debug(text)
        tb = getattr(self.log_text, "_textbox", None) or self.log_text
        self.log_text.configure(state="normal")
        for _act in actions:
            _, seg_text, tag = _act
            _log_insert_el_aware(self.log_text, tb, seg_text, tag)
        self.log_text.see("end")
        self.log_text.configure(state="disabled")

    def _do_flash(self) -> None:
        if self._flash_running:
            return
        src = self.fw_source.get()
        port = self.port_combo.get()
        s = self.app.settings if self.app else {}
        baud = s.get("baud_rate", "921600")

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
                import tempfile
                import urllib.request
                try:
                    req = urllib.request.Request(url)
                    with urllib.request.urlopen(req, timeout=120) as r:
                        tf = tempfile.NamedTemporaryFile(delete=False, suffix=".bin")
                        tf.write(r.read())
                        tf.close()
                        temp_path = tf.name
                except Exception as e:
                    return False, "Download failed: %s" % e
                try:
                    ok, msg = flash_firmware(
                        port, baud, temp_path,
                        on_line=lambda line: self.after(0, lambda l=line: self._log(l)),
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
            return flash_firmware(port, baud, path, on_line=lambda line: self.after(0, lambda l=line: self._log(l)))

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
        dev_frame.grid_columnconfigure(1, weight=1)
        ctk.CTkLabel(dev_frame, text="Port:").grid(row=0, column=0, padx=(0, 10), pady=5)
        self.port_combo = ctk.CTkComboBox(dev_frame, values=get_serial_ports(), width=220)
        self.port_combo.grid(row=0, column=1, padx=0, pady=5, sticky="w")
        ctk.CTkButton(dev_frame, text="Refresh", width=80, command=self._refresh_ports).grid(
            row=0, column=2, padx=(10, 0), pady=5
        )
        row += 1

        actions_frame = ctk.CTkFrame(self, fg_color="transparent")
        actions_frame.grid(row=row, column=0, sticky="ew", padx=20, pady=(15, 5))
        actions_frame.grid_columnconfigure(0, weight=1)
        actions_frame.grid_columnconfigure(1, weight=1)

        ctk.CTkLabel(actions_frame, text="TTGO→PC", font=ctk.CTkFont(weight="bold")).grid(
            row=0, column=0, sticky="w", padx=(0, 10), pady=(0, 6))
        ctk.CTkLabel(actions_frame, text="PC→TTGO", font=ctk.CTkFont(weight="bold")).grid(
            row=0, column=1, sticky="w", padx=(10, 0), pady=(0, 6))

        ctk.CTkButton(actions_frame, text="Make full backup", command=self._make_backup).grid(
            row=1, column=0, sticky="ew", padx=(0, 10), pady=3)
        ctk.CTkButton(actions_frame, text="Restore full backup", command=self._restore_full_backup).grid(
            row=1, column=1, sticky="ew", padx=(10, 0), pady=3)

        ctk.CTkButton(actions_frame, text="Extract filesystem").grid(
            row=2, column=0, sticky="ew", padx=(0, 10), pady=3)
        ctk.CTkButton(actions_frame, text="Upload filesystem").grid(
            row=2, column=1, sticky="ew", padx=(10, 0), pady=3)

        ctk.CTkButton(actions_frame, text="Extract filesystem from backup image").grid(
            row=3, column=0, sticky="ew", padx=(0, 10), pady=3)
        ctk.CTkButton(actions_frame, text="Setup WiFi via IMPROV").grid(
            row=3, column=1, sticky="ew", padx=(10, 0), pady=3)

        row += 1
        self._op_log = ctk.CTkTextbox(self, height=100, state="disabled")
        self._op_log.grid(row=row, column=0, sticky="nsew", padx=20, pady=(10, 20))
        self.grid_rowconfigure(row, weight=1)
        _configure_ansi_tags_for_log(self._op_log)

    def _refresh_ports(self) -> None:
        self.port_combo.configure(values=get_serial_ports())

    def _log(self, text: str) -> None:
        if not self._op_log:
            return
        actions = _parse_ansi_to_actions_debug(text)
        tb = getattr(self._op_log, "_textbox", None) or self._op_log
        self._op_log.configure(state="normal")
        for _act in actions:
            _, seg_text, tag = _act
            _log_insert_el_aware(self._op_log, tb, seg_text, tag)
        self._op_log.see("end")
        self._op_log.configure(state="disabled")

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
        baud = "115200"  # read_flash is more reliable at 115200 (matches ttgoconfig)
        self._log("\n--- Make backup to %s ---\n" % path)

        def work():
            return read_backup(port, baud, path, on_line=lambda line: self.after(0, lambda l=line: self._log(l)))

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
        baud = (self.app.settings if self.app else {}).get("baud_rate", "921600")
        self._log("\n--- Restore from %s ---\n" % path)

        def work():
            return write_backup(port, baud, path, on_line=lambda line: self.after(0, lambda l=line: self._log(l)))

        def on_done(ok: bool, _msg: str) -> None:
            if ok:
                self._log("Restore complete.\n")
                messagebox.showinfo("Restore", "Restore complete.")
            else:
                messagebox.showerror("Restore", "Restore failed. Check the log.")

        _run_in_background(work, on_done)

    def _restore_selected_files(self) -> None:
        messagebox.showinfo("Restore selected files", "Not yet implemented.")


class WiFiManageFrame(ContentFrame):
    def __init__(self, master, app: Optional["App"] = None, **kwargs):
        super().__init__(master, "Wi-Fi", app=app, **kwargs)
        self.grid_columnconfigure(0, weight=1)

        row = 0
        host_frame = ctk.CTkFrame(self, fg_color="transparent")
        host_frame.grid(row=row, column=0, sticky="ew", padx=20, pady=(10, 4))
        host_frame.grid_columnconfigure(1, weight=1)
        ctk.CTkLabel(host_frame, text="Host:").grid(row=0, column=0, sticky="w", padx=(0, 8), pady=0)
        self.host_entry = ctk.CTkEntry(host_frame, placeholder_text="rdzsonde.local or IP")
        self.host_entry.grid(row=0, column=1, sticky="ew", padx=(0, 10), pady=0)
        self.ip_label = ctk.CTkLabel(host_frame, text="", text_color="gray")
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
        self.auth_user_entry = ctk.CTkEntry(auth_frame, width=120, placeholder_text="username")
        self.auth_user_entry.grid(row=0, column=3, padx=(0, 12), pady=0, sticky="w")
        ctk.CTkLabel(auth_frame, text="Password").grid(row=0, column=4, padx=(0, 4), pady=0)
        self.auth_pass_entry = ctk.CTkEntry(auth_frame, width=120, placeholder_text="password", show="•")
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
            # remember resolved IP and show it inline next to the host entry
            self._resolved_ip = ip
            if hasattr(self, "ip_label"):
                self.ip_label.configure(text="IP: %s" % ip)
        except Exception as e:
            self._resolved_ip = ""
            if hasattr(self, "ip_label"):
                self.ip_label.configure(text="")
            messagebox.showerror("Resolve", str(e))

    def _test_connection(self) -> None:
        host = (self.host_entry.get() or "").strip() or "rdzsonde.local"
        ip = getattr(self, "_resolved_ip", "")
        if ip and host and host != ip and "://" not in host:
            base = "http://%s/" % ip
        else:
            base = self._base_url()
        url = base.rstrip("/") + "/status.json"

        def work():
            try:
                import urllib.request
                req = urllib.request.Request(url)
                with urllib.request.urlopen(req, timeout=5) as r:
                    body = r.read().decode("utf-8", errors="replace")[:500]
                return True, body
            except Exception as e:
                return False, str(e)

        def on_done(ok: bool, msg: str) -> None:
            if ok:
                messagebox.showinfo("Test connection", "Connected.\n\n%s" % (msg[:300] + "..." if len(msg) > 300 else msg))
            else:
                messagebox.showerror("Test connection", msg)

        _run_in_background(work, on_done)

    def _extract_files(self) -> None:
        folder = filedialog.askdirectory(title="Choose folder to save files")
        if not folder:
            return
        host = (self.host_entry.get() or "").strip() or "rdzsonde.local"
        try:
            import socket
            ip = socket.gethostbyname(host.split("://")[-1].split("/")[0].split(":")[0])
        except Exception as e:
            messagebox.showerror("Extract files", "Could not resolve host: %s" % e)
            return
        base_url = "http://%s/" % ip
        user = (self.auth_user_entry.get() or "").strip()
        password = (self.auth_pass_entry.get() or "").strip()
        files_to_get = ["config.txt", "qrg.txt", "networks.txt"] + ["screens%d.txt" % i for i in range(1, 6)]

        def work():
            import hashlib
            import requests
            session = requests.Session()
            if user and password:
                try:
                    r = session.get(base_url + "login.html", timeout=10)
                    r.raise_for_status()
                    m = re.search(r'name="preauth"\s+value="([^"]+)"', r.text)
                    if not m:
                        return False, "Could not get login form (preauth)."
                    preauth = m.group(1)
                    auth_hex = hashlib.sha256(("%s:%s:%s" % (user, preauth, password)).encode()).hexdigest()
                    r2 = session.post(base_url + "login.html", data={"user": user, "preauth": preauth, "auth": auth_hex}, allow_redirects=True, timeout=10)
                    if r2.status_code == 401:
                        return False, "Invalid credentials or login failed."
                except Exception as e:
                    return False, "Login failed: %s" % e
            saved = 0
            errors = []
            for name in files_to_get:
                try:
                    r = session.get(base_url + "file/" + name, timeout=10)
                    if r.status_code == 401:
                        return False, "Permission denied (401). Use Auth if the device requires login."
                    if r.status_code == 404:
                        continue
                    if r.status_code != 200:
                        errors.append("%s: HTTP %s" % (name, r.status_code))
                        continue
                    path = os.path.join(folder, name)
                    with open(path, "wb") as f:
                        f.write(r.content)
                    saved += 1
                except Exception as e:
                    errors.append("%s: %s" % (name, e))
            if errors:
                return False, "Downloaded %d file(s). Errors: %s" % (saved, "; ".join(errors[:5]))
            return True, "Downloaded %d file(s) to %s" % (saved, folder)

        def on_done(ok: bool, msg: str) -> None:
            if ok:
                messagebox.showinfo("Extract files", msg)
            else:
                messagebox.showerror("Extract files", msg)

        _run_in_background(work, on_done)

    def _sd_refresh(self) -> None:
        self.sd_refresh_btn.configure(state="disabled")
        self.sd_placeholder.configure(text="Loading…")
        self.sd_placeholder.grid()
        self.sd_scroll.grid_remove()

        host = (self.host_entry.get() or "").strip() or "rdzsonde.local"
        if "://" in host:
            base = host.rstrip("/") + "/"
            url = base.rstrip("/") + "/files.json"
        else:
            try:
                import socket
                ip = socket.gethostbyname(host)
                base = "http://%s/" % ip
                url = base + "files.json"
            except Exception as e:
                err_msg = str(e)
                def fail():
                    self.sd_placeholder.configure(text="Error: Could not resolve host. %s" % err_msg)
                    self.sd_refresh_btn.configure(state="normal")
                self.after(0, fail)
                return
        if self._sd_current_dir:
            url += "?dir=" + self._sd_current_dir

        def work():
            try:
                import urllib.request
                import urllib.error
                req = urllib.request.Request(url)
                with urllib.request.urlopen(req, timeout=8) as r:
                    if r.status != 200:
                        return False, "HTTP %s – SD card may not be available (device firmware may not have SD support)." % r.status
                    raw = r.read().decode("utf-8")
                    import json as _json
                    data = _json.loads(raw)
                return True, data
            except urllib.error.HTTPError as e:
                return False, "HTTP %s – SD card not available or device error." % e.code
            except urllib.error.URLError as e:
                err = str(e.reason) if getattr(e, "reason", None) else str(e)
                if "timed out" in err.lower():
                    return False, "Connection timed out. Check host and network."
                return False, "Connection failed: %s" % err
            except Exception as e:
                return False, "SD card not available or device error: %s" % e

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
        for i, item in enumerate(self._sd_entries):
            name = item.get("name", "?")
            is_dir = bool(item.get("dir"))
            size = item.get("size", 0)
            row_f = ctk.CTkFrame(self.sd_scroll, fg_color="transparent")
            row_f.grid(row=i, column=0, sticky="w", pady=1)
            var = ctk.BooleanVar(value=False)
            ctk.CTkCheckBox(row_f, text="", variable=var, width=24).grid(row=0, column=0, padx=(0, 6), pady=2)
            if is_dir:
                label_text = "[DIR]  %s/" % name
            else:
                label_text = "%s  (%s bytes)" % (name, size)
            lbl = ctk.CTkLabel(row_f, text=label_text, anchor="w")
            lbl.grid(row=0, column=1, sticky="w", padx=0, pady=2)
            if is_dir:
                def make_enter(d=self._sd_current_dir, n=name):
                    def enter():
                        self._sd_current_dir = (d + "/" + n).strip("/") if d else n
                        self._sd_refresh()
                    return enter
                btn = ctk.CTkButton(row_f, text="Open", width=50, command=make_enter())
                btn.grid(row=0, column=2, padx=(8, 0), pady=2)
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
        try:
            import socket
            ip = socket.gethostbyname(host.split("://")[-1].split("/")[0].split(":")[0])
        except Exception:
            ip = host
        base_url = "http://%s/" % ip
        import requests
        session = requests.Session()
        user = (self.auth_user_entry.get() or "").strip()
        password = (self.auth_pass_entry.get() or "").strip()
        if user and password:
            import hashlib
            r = session.get(base_url + "login.html", timeout=10)
            r.raise_for_status()
            m = re.search(r'name="preauth"\s+value="([^"]+)"', r.text)
            if m:
                preauth = m.group(1)
                auth_hex = hashlib.sha256(("%s:%s:%s" % (user, preauth, password)).encode()).hexdigest()
                session.post(base_url + "login.html", data={"user": user, "preauth": preauth, "auth": auth_hex}, timeout=10)
        return session, base_url

    def _sd_collect_files_recursive(self, session: Any, base_url: str, dir_path: str) -> list[str]:
        """Return list of SD paths (dir_path/name) for all files under dir_path."""
        import json as _json
        url = base_url + "files.json"
        if dir_path:
            url += "?dir=" + dir_path
        out: list[str] = []
        try:
            r = session.get(url, timeout=8)
            if r.status_code != 200:
                return out
            data = r.json()
        except Exception:
            return out
        for item in (data or []):
            name = item.get("name", "")
            if not name:
                continue
            full = (dir_path + "/" + name).strip("/") if dir_path else name
            if item.get("dir"):
                out.extend(self._sd_collect_files_recursive(session, base_url, full))
            else:
                out.append(full)
        return out

    def _sd_download_all(self) -> None:
        dest = filedialog.askdirectory(title="Choose folder to save SD files")
        if not dest:
            return
        session, base_url = self._sd_get_session_and_base()

        def work(progress_cb=None):
            files = self._sd_collect_files_recursive(session, base_url, self._sd_current_dir)
            total = len(files)
            saved = 0
            errs = []
            for i, path in enumerate(files):
                try:
                    r = session.get(base_url + "sd/" + path, timeout=15)
                    if r.status_code != 200:
                        errs.append("%s: HTTP %s" % (path, r.status_code))
                        continue
                    local = os.path.join(dest, path)
                    os.makedirs(os.path.dirname(local) or ".", exist_ok=True)
                    with open(local, "wb") as f:
                        f.write(r.content)
                    saved += 1
                except Exception as e:
                    errs.append("%s: %s" % (path, e))
                if progress_cb and total:
                    progress_cb(i + 1, total)
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
        session, base_url = self._sd_get_session_and_base()

        def work(progress_cb=None):
            all_paths: list[str] = []
            for _checked, entry in selected:
                name = entry.get("name", "")
                if not name:
                    continue
                full = (self._sd_current_dir + "/" + name).strip("/") if self._sd_current_dir else name
                if entry.get("dir"):
                    all_paths.extend(self._sd_collect_files_recursive(session, base_url, full))
                else:
                    all_paths.append(full)
            total = len(all_paths)
            saved = 0
            errs = []
            for i, path in enumerate(all_paths):
                try:
                    r = session.get(base_url + "sd/" + path, timeout=15)
                    if r.status_code != 200:
                        errs.append("%s: HTTP %s" % (path, r.status_code))
                        continue
                    local = os.path.join(dest, path)
                    os.makedirs(os.path.dirname(local) or ".", exist_ok=True)
                    with open(local, "wb") as f:
                        f.write(r.content)
                    saved += 1
                except Exception as e:
                    errs.append("%s: %s" % (path, e))
                if progress_cb and total:
                    progress_cb(i + 1, total)
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
        self.host_entry = ctk.CTkEntry(host_frame, placeholder_text="rdzsonde.local or IP")
        self.host_entry.grid(row=0, column=1, sticky="ew", padx=(0, 10), pady=0)
        self.ip_label = ctk.CTkLabel(host_frame, text="", text_color="gray")
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
        self.auth_user_entry = ctk.CTkEntry(auth_frame, width=120, placeholder_text="username")
        self.auth_user_entry.grid(row=0, column=3, padx=(0, 12), pady=0, sticky="w")
        ctk.CTkLabel(auth_frame, text="Password").grid(row=0, column=4, padx=(0, 4), pady=0)
        self.auth_pass_entry = ctk.CTkEntry(auth_frame, width=120, placeholder_text="password", show="•")
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
        dev_frame.grid_columnconfigure(1, weight=1)
        ctk.CTkLabel(dev_frame, text="Port:").grid(row=0, column=0, padx=(0, 10), pady=5)
        self.port_combo = ctk.CTkComboBox(dev_frame, values=get_serial_ports(), width=220)
        self.port_combo.grid(row=0, column=1, padx=0, pady=5, sticky="w")
        ctk.CTkLabel(dev_frame, text="Baud:").grid(row=0, column=2, padx=(20, 8), pady=5)
        self.baud_combo = ctk.CTkComboBox(
            dev_frame, values=["115200", "9600", "19200", "38400", "57600", "230400", "460800", "921600"],
            width=100,
        )
        self.baud_combo.set("115200")
        self.baud_combo.grid(row=0, column=3, padx=0, pady=5, sticky="w")
        ctk.CTkButton(dev_frame, text="Refresh", width=80, command=self._refresh_ports).grid(
            row=0, column=4, padx=(10, 0), pady=5
        )
        self.connect_btn = ctk.CTkButton(dev_frame, text="Connect", width=90, command=self._toggle_connect)
        self.connect_btn.grid(row=0, column=5, padx=(10, 0), pady=5)
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
        ctk.CTkCheckBox(bot, text="Strip ANSI when saving", variable=self.strip_ansi_var).grid(
            row=0, column=1, padx=10, pady=0
        )
        ctk.CTkButton(bot, text="Clear", width=80, command=self._clear_log).grid(row=0, column=2, padx=10, pady=0)
        ctk.CTkButton(bot, text="Copy", width=80, command=self._copy_log).grid(row=0, column=3, padx=0, pady=0)

    def _refresh_ports(self) -> None:
        self.port_combo.configure(values=get_serial_ports())

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
        """Drain serial queue and append to log on main thread; reschedule if still connected."""
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
        if self._serial and self._serial.is_open:
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
        ctk.CTkLabel(self, text="Backups", font=ctk.CTkFont(weight="bold")).grid(
            row=row, column=0, columnspan=3, sticky="w", padx=20, pady=(20, 8))
        row += 1
        ctk.CTkLabel(self, text="Backup folder:", width=LABEL_W, anchor="w").grid(
            row=row, column=0, sticky="w", padx=(20, 10), pady=4)
        self.backup_folder_var = ctk.StringVar(value="")
        self.backup_entry = ctk.CTkEntry(
            self, textvariable=self.backup_folder_var, placeholder_text="e.g. ~/rdzTTGOsonde/backups", width=ENTRY_MIN_W
        )
        self.backup_entry.grid(row=row, column=1, sticky="ew", padx=0, pady=4)
        ctk.CTkButton(self, text="Browse…", width=80, command=self._browse_backup).grid(row=row, column=2, padx=(10, 20), pady=4)
        row += 1

        ctk.CTkLabel(self, text="Firmware", font=ctk.CTkFont(weight="bold")).grid(
            row=row, column=0, columnspan=3, sticky="w", padx=20, pady=(16, 8))
        row += 1
        ctk.CTkLabel(self, text="Download server URL:", width=LABEL_W, anchor="w").grid(
            row=row, column=0, sticky="w", padx=(20, 10), pady=4)
        self.download_url_var = ctk.StringVar(value="")
        ctk.CTkEntry(self, textvariable=self.download_url_var, placeholder_text="https://rdzsonde.org", width=ENTRY_MIN_W).grid(
            row=row, column=1, sticky="ew", padx=0, pady=4)
        row += 1

        ctk.CTkLabel(self, text="Serial (advanced)", font=ctk.CTkFont(weight="bold")).grid(
            row=row, column=0, columnspan=3, sticky="w", padx=20, pady=(16, 8))
        row += 1
        ctk.CTkLabel(self, text="Default baud:", width=LABEL_W, anchor="w").grid(
            row=row, column=0, sticky="w", padx=(20, 10), pady=4)
        self.baud_var = ctk.StringVar(value="921600")
        ctk.CTkComboBox(self, values=["921600", "115200", "460800"], variable=self.baud_var, width=120).grid(
            row=row, column=1, sticky="w", padx=0, pady=4)
        row += 1
        ctk.CTkLabel(self, text="Serial log", font=ctk.CTkFont(weight="bold")).grid(
            row=row, column=0, columnspan=3, sticky="w", padx=20, pady=(16, 8))
        row += 1
        ctk.CTkLabel(self, text="Scroll-back buffer (lines):", width=LABEL_W, anchor="w").grid(
            row=row, column=0, sticky="w", padx=(20, 10), pady=4)
        self.serial_log_lines_var = ctk.StringVar(value="10000")
        ctk.CTkEntry(self, textvariable=self.serial_log_lines_var, width=100, placeholder_text="10000").grid(
            row=row, column=1, sticky="w", padx=0, pady=4)
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
        self.baud_var.set(s.get("baud_rate", settings_mod.DEFAULT_BAUD))
        self.serial_log_lines_var.set(str(s.get("serial_log_buffer_lines", settings_mod.DEFAULT_SERIAL_LOG_LINES)))

    def _save_from_ui(self) -> None:
        if not self.app:
            return
        try:
            n = int(self.serial_log_lines_var.get().strip() or "10000")
        except ValueError:
            n = settings_mod.DEFAULT_SERIAL_LOG_LINES
        self.app.settings.update({
            "backup_folder": self.backup_folder_var.get().strip() or settings_mod.DEFAULT_BACKUP_FOLDER,
            "download_url": self.download_url_var.get().strip() or settings_mod.DEFAULT_DOWNLOAD_URL,
            "baud_rate": self.baud_var.get(),
            "serial_log_buffer_lines": n,
        })
        settings_mod.save(self.app.settings)


class App(ctk.CTk):
    def __init__(self):
        super().__init__()
        self.title("rdzTTGOsonde Flasher")
        self.geometry("720x520")
        self.minsize(500, 400)

        self.settings = settings_mod.load()

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
            "(Use the same Python/venv you use to run this app, e.g. from installer folder or project root.)"
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

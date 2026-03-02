"""
rdzTTGOsonde Flasher – CLI (mirrors GUI: flash, serial, USB, Wi‑Fi, SD).

Run: python -m ttgoconfig.cli
"""

from __future__ import annotations

import argparse
import os
import re
import sys
from datetime import datetime
from typing import Optional

import tempfile

from . import settings as settings_mod
from .esptool_helper import (
    flash_app_partition,
    flash_firmware,
    read_backup,
    download_filesystem,
    upload_filesystem,
    PARTITION_SPIFFS_OFFSET,
    PARTITION_SPIFFS_SIZE,
    set_no_stub,
)
from .littlefs_helper import (
    pack_directory,
    unpack_image,
    extract_from_backup,
    BLOCK_SIZE as LFS_BLOCK_SIZE,
)
from .firmware import (
    list_available_versions,
    resolve_firmware_url,
    download_firmware_to_temp,
)
from .wifi_ops import (
    FILE_SETS,
    session_for_host,
    get_file,
    put_file,
    get_files_by_kind,
    test_connection,
)
from .sd_ops import (
    list_dir,
    resolve_sd_path_to_list,
    fetch_paths_to_dir,
)


def _strip_ansi(s: str) -> str:
    s = re.sub(r"\x1b\[[0-9;]*[a-zA-Z]", "", s)
    s = re.sub(r"\x1b\[K", "", s)
    return s


def _port_value(port: Optional[str]) -> Optional[str]:
    if not port or port == "Auto" or (isinstance(port, str) and port.startswith("(")):
        return None
    return port


def cmd_flash(args: argparse.Namespace) -> int:
    base_url = (getattr(args, "download_url", None) or "").strip() or settings_mod.DEFAULT_DOWNLOAD_URL
    port = _port_value(getattr(args, "port", None))
    baud = getattr(args, "baud", None) or settings_mod.DEFAULT_BAUD
    part = (getattr(args, "part", None) or "full").lower()

    image_path = None
    if getattr(args, "file", None):
        fp = args.file
        if not os.path.isfile(fp):
            print("Error: File not found:", fp, file=sys.stderr)
            return 1
        image_path = fp
    elif getattr(args, "install", False):
        versions, err = list_available_versions(base_url)
        if err:
            print("Error:", err, file=sys.stderr)
            return 1
        print("Available versions:")
        for d, u in versions:
            print("  ", d)
        return 0
    elif getattr(args, "download", None) or getattr(args, "name", None):
        name = args.download or getattr(args, "name", None)
        if not name:
            print("Error: Specify --download NAME, --file PATH, or --install", file=sys.stderr)
            return 1
        if os.path.isfile(name):
            image_path = name
        else:
            url, err = resolve_firmware_url(base_url, name)
            if err:
                print("Error:", err, file=sys.stderr)
                return 1
            temp_path, err = download_firmware_to_temp(url)
            if err:
                print("Error:", err, file=sys.stderr)
                return 1
            try:
                if part == "code":
                    ok, msg = flash_app_partition(port, baud, temp_path, on_line=lambda l: print(l, end=""))
                else:
                    ok, msg = flash_firmware(port, baud, temp_path, on_line=lambda l: print(l, end=""))
                if not ok:
                    print("Error:", msg, file=sys.stderr)
                    return 1
                print("Flashed successfully.")
                return 0
            finally:
                try:
                    os.unlink(temp_path)
                except Exception:
                    pass
    else:
        print("Error: Specify --download NAME, --file PATH, or --install", file=sys.stderr)
        return 1

    if not image_path:
        return 1
    if part == "code":
        ok, msg = flash_app_partition(port, baud, image_path, on_line=lambda l: print(l, end=""))
    else:
        ok, msg = flash_firmware(port, baud, image_path, on_line=lambda l: print(l, end=""))
    if not ok:
        print("Error:", msg, file=sys.stderr)
        return 1
    print("Flashed successfully.")
    return 0


def cmd_serial(args: argparse.Namespace) -> int:
    try:
        import serial
    except ImportError:
        print("Error: pyserial not installed. pip install pyserial", file=sys.stderr)
        return 1
    port = _port_value(getattr(args, "port", None))
    if not port:
        print("Error: Serial port required (e.g. --port /dev/cu.usbserial-0001)", file=sys.stderr)
        return 1
    baud = int(getattr(args, "baud", "115200"))
    strip = getattr(args, "stripansi", False)
    save_path = getattr(args, "save", None)
    try:
        ser = serial.Serial(port, baud, timeout=0.1)
    except Exception as e:
        print("Error opening port:", e, file=sys.stderr)
        return 1
    fout = open(save_path, "ab") if save_path else None
    try:
        while True:
            line = ser.readline()
            if line:
                text = line.decode("utf-8", errors="replace")
                if strip:
                    text = _strip_ansi(text)
                print(text, end="")
                if fout:
                    fout.write(line)
                    fout.flush()
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
        if fout:
            fout.close()
    return 0


def cmd_backup(args: argparse.Namespace) -> int:
    port = _port_value(getattr(args, "port", None))
    baud = getattr(args, "baud", None) or settings_mod.DEFAULT_BAUD_READ
    folder = getattr(args, "dir", None) or settings_mod.get_backup_folder_expanded(settings_mod.load())
    os.makedirs(folder, exist_ok=True)
    if getattr(args, "file", None):
        out_path = args.file
    else:
        out_path = os.path.join(folder, "backup-%s.bin" % datetime.now().strftime("%Y%m%d-%H%M"))
    ok, msg = read_backup(port, baud, out_path, on_line=lambda l: print(l, end=""))
    if not ok:
        print("Error:", msg, file=sys.stderr)
        return 1
    print("Backup saved to", out_path)
    return 0


def _improv_port_and_baud(args: argparse.Namespace) -> tuple:
    port = _port_value(getattr(args, "port", None))
    if not port:
        saved = settings_mod.load()
        port = (saved.get("last_port") or "").strip()
    if not port:
        print("Error: No serial port. Use --port or set last_port in settings.", file=sys.stderr)
        return None, None
    baud = int(getattr(args, "baud", None) or settings_mod.DEFAULT_BAUD_SERIAL)
    return port, baud


def cmd_improv(args: argparse.Namespace) -> int:
    action = getattr(args, "improv_action", None)
    if not action:
        print("Error: Use improv info | improv wifiscan | improv connect <ssid> <password>", file=sys.stderr)
        return 1
    port, baud = _improv_port_and_baud(args)
    if port is None:
        return 1
    try:
        import serial
    except ImportError:
        print("Error: pyserial not installed. pip install pyserial", file=sys.stderr)
        return 1
    from . import improv_serial as improv_mod
    try:
        ser = serial.Serial(port, baud, timeout=0.5)
    except Exception as e:
        print("Error opening %s: %s" % (port, e), file=sys.stderr)
        return 1
    try:
        if action == "info":
            info = improv_mod.rpc_get_info(ser)
            if info is None:
                print("IMPROV info: no response (device may not support IMPROV or wrong port/baud).", file=sys.stderr)
                return 1
            for s in info:
                print(s)
            return 0
        if action == "wifiscan":
            networks = improv_mod.rpc_wifi_scan(ser)
            if not networks:
                print("No networks or no response.")
            else:
                for ssid, rssi, secured in networks:
                    print("%s  rssi=%s  %s" % (ssid, rssi, secured))
            return 0
        if action == "connect":
            ssid = getattr(args, "ssid", None) or ""
            password = getattr(args, "password", None) or ""
            if not ssid:
                print("Error: improv connect requires SSID and password.", file=sys.stderr)
                return 1
            result = improv_mod.rpc_send_wifi(ser, ssid, password)
            if result is None:
                print("IMPROV connect: no success response.", file=sys.stderr)
                return 1
            print("Provisioned. URL: %s" % result)
            return 0
        print("Error: unknown improv action %s" % action, file=sys.stderr)
        return 1
    finally:
        try:
            ser.close()
        except Exception:
            pass


def cmd_uploadfs(args: argparse.Namespace) -> int:
    port = _port_value(getattr(args, "port", None))
    baud = getattr(args, "baud", None) or settings_mod.DEFAULT_BAUD_WRITE
    src_dir = args.directory
    keep_image = getattr(args, "keep_image", False)
    if not os.path.isdir(src_dir):
        print("Error: '%s' is not a directory." % src_dir, file=sys.stderr)
        return 1
    block_count = PARTITION_SPIFFS_SIZE // LFS_BLOCK_SIZE
    tmp_fd, tmp_path = tempfile.mkstemp(suffix=".bin", prefix="lfs_upload_")
    os.close(tmp_fd)
    try:
        print("Packing '%s' into LittleFS image (%d blocks × 4096 B)…" % (src_dir, block_count))
        pack_directory(src_dir, tmp_path, block_count)
        image_size = os.path.getsize(tmp_path)
        print("Image built: %d bytes → %s" % (image_size, tmp_path))
        print("Flashing filesystem to device…")
        ok, msg = upload_filesystem(port, baud, tmp_path, on_line=lambda l: print(l, end=""))
        if not ok:
            print("Error:", msg, file=sys.stderr)
            return 1
        print("Filesystem uploaded successfully.")
        return 0
    finally:
        if keep_image:
            print("Image kept at: %s" % tmp_path)
        else:
            try:
                os.unlink(tmp_path)
            except Exception:
                pass


def cmd_downloadfs(args: argparse.Namespace) -> int:
    port = _port_value(getattr(args, "port", None))
    baud = getattr(args, "baud", None) or settings_mod.DEFAULT_BAUD_READ
    dest_dir = args.directory
    block_count = PARTITION_SPIFFS_SIZE // LFS_BLOCK_SIZE
    keep_image = getattr(args, "keep_image", False)
    tmp_fd, tmp_path = tempfile.mkstemp(suffix=".bin", prefix="lfs_download_")
    os.close(tmp_fd)
    success = False
    try:
        print("Reading filesystem from device…")
        ok, msg = download_filesystem(port, baud, tmp_path, on_line=lambda l: print(l, end=""))
        if not ok:
            print("Error:", msg, file=sys.stderr)
            return 1
        image_size = os.path.getsize(tmp_path)
        print("Downloaded %d bytes (block_count=%d, block_size=%d)" % (image_size, block_count, LFS_BLOCK_SIZE))
        os.makedirs(dest_dir, exist_ok=True)
        print("Extracting LittleFS image to '%s'…" % dest_dir)
        unpack_image(tmp_path, dest_dir, block_count)
        print("Filesystem extracted successfully.")
        success = True
        return 0
    finally:
        if keep_image or not success:
            print("Image kept at: %s" % tmp_path)
        else:
            try:
                os.unlink(tmp_path)
            except Exception:
                pass


def cmd_extractfs(args: argparse.Namespace) -> int:
    image_path = args.image
    dest_dir = args.directory
    if not os.path.isfile(image_path):
        print("Error: '%s' not found." % image_path, file=sys.stderr)
        return 1
    os.makedirs(dest_dir, exist_ok=True)
    print("Extracting filesystem from backup '%s' to '%s'…" % (image_path, dest_dir))
    try:
        extract_from_backup(image_path, dest_dir, PARTITION_SPIFFS_OFFSET, PARTITION_SPIFFS_SIZE)
        print("Filesystem extracted successfully.")
        return 0
    except Exception as e:
        print("Error:", e, file=sys.stderr)
        return 1


def cmd_wifi(args: argparse.Namespace) -> int:
    host = getattr(args, "ttgo", None) or "rdzsonde.local"
    user = getattr(args, "user", None)
    password = getattr(args, "password", None)
    session, base_url, err = session_for_host(host, user, password)
    if err:
        print("Error:", err, file=sys.stderr)
        return 1

    action = getattr(args, "wifi_action", None)
    if action == "test":
        ok, msg = test_connection(host)
        if ok:
            print(msg)
            return 0
        print("Error:", msg, file=sys.stderr)
        return 1

    if action == "get":
        kind = getattr(args, "kind", None)
        if kind:
            filenames, err = get_files_by_kind(session, base_url, kind)
            if err:
                print("Error:", err, file=sys.stderr)
                return 1
            outdir = getattr(args, "dir", None) or os.getcwd()
            os.makedirs(outdir, exist_ok=True)
            for name in filenames:
                content, err = get_file(session, base_url, name)
                if err:
                    print("Error %s: %s" % (name, err), file=sys.stderr)
                    continue
                path = os.path.join(outdir, name)
                with open(path, "wb") as f:
                    f.write(content)
                print("Downloaded", path)
            return 0
        filename = getattr(args, "filename", None)
        if not filename:
            print("Error: get requires <filename> or --kind <setname>", file=sys.stderr)
            return 1
        content, err = get_file(session, base_url, filename)
        if err:
            print("Error:", err, file=sys.stderr)
            return 1
        outdir = getattr(args, "dir", None) or os.getcwd()
        path = os.path.join(outdir, os.path.basename(filename))
        with open(path, "wb") as f:
            f.write(content)
        print("Downloaded", path)
        return 0

    if action == "put":
        kind = getattr(args, "kind", None)
        if kind:
            filenames, err = get_files_by_kind(session, base_url, kind)
            if err:
                print("Error:", err, file=sys.stderr)
                return 1
            indir = getattr(args, "dir", None) or os.getcwd()
            for name in filenames:
                path = os.path.join(indir, name)
                err = put_file(session, base_url, path, name)
                if err:
                    print("Error %s: %s" % (name, err), file=sys.stderr)
                    continue
                print("Uploaded", name)
            return 0
        filename = getattr(args, "filename", None)
        if not filename:
            print("Error: put requires <filename> or --kind <setname>", file=sys.stderr)
            return 1
        path = filename if os.path.isfile(filename) else os.path.join(getattr(args, "dir", None) or os.getcwd(), os.path.basename(filename))
        err = put_file(session, base_url, path, os.path.basename(filename))
        if err:
            print("Error:", err, file=sys.stderr)
            return 1
        print("Uploaded", filename)
        return 0

    print("Error: wifi requires get, put, or test", file=sys.stderr)
    return 1


def cmd_sd(args: argparse.Namespace) -> int:
    host = getattr(args, "ttgo", None) or "rdzsonde.local"
    user = getattr(args, "user", None)
    password = getattr(args, "password", None)
    session, base_url, err = session_for_host(host, user, password)
    if err:
        print("Error:", err, file=sys.stderr)
        return 1

    sd_action = getattr(args, "sd_action", None)
    if sd_action == "list":
        dir_path = getattr(args, "dir_path", "") or ""
        entries, err = list_dir(session, base_url, dir_path)
        if err:
            print("Error:", err, file=sys.stderr)
            return 1
        for item in entries:
            name = item.get("name", "?")
            is_dir = item.get("dir", False)
            size = item.get("size", 0)
            if is_dir:
                print("[DIR]  %s/" % name)
            else:
                print("%s  (%s bytes)" % (name, size))
        return 0

    if sd_action == "fetch":
        path = getattr(args, "path", None)
        if not path:
            print("Error: sd fetch requires <file_or_dir>", file=sys.stderr)
            return 1
        paths, err = resolve_sd_path_to_list(session, base_url, path)
        if err:
            print("Error:", err, file=sys.stderr)
            return 1
        outdir = getattr(args, "outdir", None) or os.getcwd()
        saved, errors = fetch_paths_to_dir(session, base_url, paths, outdir)
        if errors:
            for e in errors[:10]:
                print("Error:", e, file=sys.stderr)
        print("Downloaded %d file(s) to %s" % (saved, outdir))
        return 0 if not errors else 1

    print("Error: sd requires list or fetch", file=sys.stderr)
    return 1


def main() -> int:
    parser = argparse.ArgumentParser(
        prog="ttgoconfig",
        description="rdzTTGOsonde Flasher CLI – flash, serial, backup, Wi‑Fi, SD.",
    )
    parser.add_argument("--port", default=None, help="Serial port (e.g. /dev/cu.usbserial-0001)")
    parser.add_argument("--ttgo", default="rdzsonde.local", help="Device host (IP or mDNS) for Wi‑Fi/SD")
    parser.add_argument("--user", default=None, help="HTTP auth user")
    parser.add_argument("--pass", dest="password", default=None, help="HTTP auth password")
    parser.add_argument("--dir", default=None, help="Directory for get/put/backup")
    parser.add_argument("--file", default=None, help="File path (backup output or flash input)")
    parser.add_argument("--outdir", default=None, help="Output directory for fetch")
    parser.add_argument("--baud", default=None, help="Baud rate (default from settings or 921600)")
    parser.add_argument("--download-url", default=None, help="Firmware download base URL")
    parser.add_argument(
        "--no-stub", dest="no_stub", action="store_true",
        help="Pass --no-stub to esptool (needed for some ESP32-C3/S3 variants)",
    )

    sub = parser.add_subparsers(dest="command", metavar="COMMAND")

    p_flash = sub.add_parser("flash", help="Flash firmware")
    p_flash.add_argument("--download", metavar="NAME", help="Download and flash: main, dev2, or pattern")
    p_flash.add_argument("--install", action="store_true", help="List available versions only")
    p_flash.add_argument("--file", metavar="PATH", help="Flash from local file or backup")
    p_flash.add_argument("--part", default="full", help="Partition: full (default) or code")
    p_flash.add_argument("name", nargs="?", help="Shortcut: main|dev2|path|pattern")
    p_flash.set_defaults(func=cmd_flash)

    p_serial = sub.add_parser("serial", help="Serial console (Ctrl+C to exit)")
    p_serial.add_argument("baud", nargs="?", default="115200", help="Baud rate")
    p_serial.add_argument("--port", default=None)
    p_serial.add_argument("--save", metavar="FILE", help="Continuously append output to file")
    p_serial.add_argument("--stripansi", action="store_true", help="Strip ANSI escape codes")
    p_serial.set_defaults(func=cmd_serial)

    p_backup = sub.add_parser("backup", help="Make full backup from device")
    p_backup.add_argument("--port", default=None)
    p_backup.add_argument("--file", metavar="PATH", help="Output file (default: backup-YYYYMMDD-HHMM.bin)")
    p_backup.add_argument("--dir", metavar="DIR", help="Output directory (default: settings backup folder)")
    p_backup.set_defaults(func=cmd_backup)

    p_improv = sub.add_parser("improv", help="IMPROV over serial: info, wifiscan, connect")
    p_improv.add_argument("--port", default=None, help="Serial port (or use last_port from settings)")
    p_improv.add_argument("--baud", default=None, help="Baud rate (default 115200)")
    p_improv_sub = p_improv.add_subparsers(dest="improv_action", metavar="info|wifiscan|connect")
    p_improv_info = p_improv_sub.add_parser("info", help="Query device name and version")
    p_improv_info.set_defaults(improv_action="info")
    p_improv_wifiscan = p_improv_sub.add_parser("wifiscan", help="List available WiFi networks")
    p_improv_wifiscan.set_defaults(improv_action="wifiscan")
    p_improv_connect = p_improv_sub.add_parser("connect", help="Provision device: connect to SSID with password")
    p_improv_connect.add_argument("ssid", help="WiFi SSID")
    p_improv_connect.add_argument("password", nargs="?", default="", help="WiFi password")
    p_improv_connect.set_defaults(improv_action="connect")
    p_improv.set_defaults(func=cmd_improv)

    p_uploadfs = sub.add_parser("uploadfs", help="Upload filesystem to device")
    p_uploadfs.add_argument("--port", default=None)
    p_uploadfs.add_argument("directory", help="Local directory to upload")
    p_uploadfs.add_argument("--keep-image", dest="keep_image", action="store_true",
                            help="Keep the generated LittleFS image after flashing (useful for debugging)")
    p_uploadfs.set_defaults(func=cmd_uploadfs)

    p_downloadfs = sub.add_parser("downloadfs", help="Extract filesystem from device")
    p_downloadfs.add_argument("--port", default=None)
    p_downloadfs.add_argument("directory", help="Local directory to save to")
    p_downloadfs.add_argument("--keep-image", dest="keep_image", action="store_true",
                              help="Keep the raw flash image after extraction (useful for debugging)")
    p_downloadfs.set_defaults(func=cmd_downloadfs)

    p_extractfs = sub.add_parser("extractfs", help="Extract filesystem from backup image")
    p_extractfs.add_argument("directory", help="Local directory to save to")
    p_extractfs.add_argument("--image", required=True, metavar="BACKUP.bin", help="Backup image file")
    p_extractfs.set_defaults(func=cmd_extractfs)

    p_wifi = sub.add_parser("wifi", help="Wi‑Fi file get/put/test")
    p_wifi.add_argument("--ttgo", default="rdzsonde.local")
    p_wifi.add_argument("--user", default=None)
    p_wifi.add_argument("--pass", dest="password", default=None)
    p_wifi.add_argument("--dir", default=None, help="Directory for get/put")
    p_wifi.add_argument("wifi_action", choices=["get", "put", "test"], metavar="get|put|test")
    p_wifi.add_argument("filename", nargs="?", help="Single filename (for get/put)")
    p_wifi.add_argument("--kind", choices=list(FILE_SETS.keys()), help="Set name: config, qrg, wifi, screens, allconfig, all")
    p_wifi.set_defaults(func=cmd_wifi)

    p_sd = sub.add_parser("sd", help="SD card over Wi‑Fi")
    p_sd.add_argument("--ttgo", default="rdzsonde.local")
    p_sd.add_argument("--user", default=None)
    p_sd.add_argument("--pass", dest="password", default=None)
    p_sd.add_argument("--outdir", default=None, help="Output directory for fetch")
    p_sd_sub = p_sd.add_subparsers(dest="sd_action", metavar="list|fetch")
    p_sd_list = p_sd_sub.add_parser("list", help="List SD directory")
    p_sd_list.add_argument("dir_path", nargs="?", default="", help="SD path (default: root)")
    p_sd_list.set_defaults(sd_action="list")
    p_sd_fetch = p_sd_sub.add_parser("fetch", help="Fetch file or directory")
    p_sd_fetch.add_argument("path", help="SD file or directory path")
    p_sd_fetch.add_argument("--outdir", default=None)
    p_sd_fetch.set_defaults(sd_action="fetch")
    p_sd.set_defaults(func=cmd_sd)

    args = parser.parse_args()

    # Apply --no-stub before any esptool call
    if getattr(args, "no_stub", False):
        set_no_stub(True)
    else:
        # Fall back to saved setting
        saved = settings_mod.load()
        if saved.get("no_stub", False):
            set_no_stub(True)

    if not args.command:
        parser.print_help()
        return 0
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())

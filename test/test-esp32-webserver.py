#!/usr/bin/env python3
"""Live ESP32 :80 web-interface smoke / soak tests (gsnext-rdz-plainidf).

Usage:
  python3 test/test-esp32-webserver.py --host 192.168.4.1
  python3 test/test-esp32-webserver.py --host 10.0.0.12 --loop --interval 20
  python3 test/test-esp32-webserver.py --host IP --only "get /,get /status.json"

# TODO (not implemented):
# - OTA: POST /update.html (do not POST firmware from this harness)
# - control/button: POST /control.html (rx, scan, reboot, …)
# - config.html form POST (distinct from PUT /file/config.txt)
# - /sdrm.html delete (opt-in only)
# - legacy leftovers: /live.kml, /dynamic.kml, *.gpx
# - --load 1..3 extra parallel/idle sessions (device max_open_sockets=7)
"""

from __future__ import annotations

import argparse
import hashlib
import http.client
import json
import re
import sys
import time
import urllib.parse
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_DATA = REPO_ROOT / "RX_FSK" / "data"

# Seed files compared via GET /file/<name> (exclude suite-mutated names).
MUTATED = frozenset({"config.txt", "qrg.txt", "networks.txt"})
LFS_COMPARE = (
    "style.css",
    "rdz.js",
    "cfg.js",
    "sha256.min.js",
    "edit.html",
    "sdfiles.html",
    "map.html",
    "livemap.js",
    "decoder.html",
    "decoder.js",
    "screens.txt",
    "screens1.txt",
    "screens2.txt",
    "screens3.txt",
    "screens4.txt",
    "screens5.txt",
    "screens6.txt",
    "user.txt",
    "gpsinit.txt",
)
STATIC_COMPARE = (
    "style.css",
    "rdz.js",
    "cfg.js",
    "sha256.min.js",
    "edit.html",
    "sdfiles.html",
    "map.html",
    "livemap.js",
    "decoder.html",
    "decoder.js",
)

# ---------------------------------------------------------------------------
# HTTP helper — one connection per case (no keepalive across cases)
# ---------------------------------------------------------------------------


class Ctx:
    def __init__(self, host: str, port: int, data_dir: Path, timeout: float = 15.0):
        self.host = host
        self.port = port
        self.data_dir = data_dir
        self.timeout = timeout
        self._conn: http.client.HTTPConnection | None = None

    def _ensure(self) -> http.client.HTTPConnection:
        if self._conn is None:
            self._conn = http.client.HTTPConnection(self.host, self.port, timeout=self.timeout)
        return self._conn

    def close(self) -> None:
        if self._conn is not None:
            try:
                self._conn.close()
            except Exception:
                pass
            self._conn = None

    def request(
        self,
        method: str,
        path: str,
        body: bytes | None = None,
        headers: dict | None = None,
        timed: bool = True,
    ):
        """Return (status, headers_dict, body_bytes, rtt_ms|None)."""
        hdrs = dict(headers or {})
        if body is not None and "Content-Length" not in hdrs:
            hdrs["Content-Length"] = str(len(body))
        conn = self._ensure()
        t0 = time.perf_counter()
        try:
            conn.request(method, path, body=body, headers=hdrs)
            resp = conn.getresponse()
            data = resp.read()
            # Build multi-value-friendly header dict (Set-Cookie may repeat)
            rh: dict[str, str] = {}
            for k, v in resp.getheaders():
                lk = k.lower()
                if lk in rh:
                    rh[lk] = rh[lk] + "\n" + v
                else:
                    rh[lk] = v
            status = resp.status
        except Exception as e:
            self.close()
            rtt = (time.perf_counter() - t0) * 1000.0 if timed else None
            raise RuntimeError(f"{method} {path}: {e}") from e
        rtt = (time.perf_counter() - t0) * 1000.0 if timed else None
        return status, rh, data, rtt

    def get(self, path: str, timed: bool = True):
        return self.request("GET", path, timed=timed)

    def put(self, path: str, body: bytes, content_type: str = "text/plain; charset=utf-8", timed: bool = True):
        return self.request(
            "PUT", path, body=body, headers={"Content-Type": content_type}, timed=timed
        )

    def post_form(self, path: str, fields: dict, timed: bool = True):
        body = urllib.parse.urlencode(fields).encode("utf-8")
        return self.request(
            "POST",
            path,
            body=body,
            headers={"Content-Type": "application/x-www-form-urlencoded"},
            timed=timed,
        )


# ---------------------------------------------------------------------------
# Tiny helpers used by cases
# ---------------------------------------------------------------------------


def _ok(rtt, msg=""):
    return "ok", rtt, msg


def _fail(rtt, msg):
    return "fail", rtt, msg


def _skip(msg):
    return "skip", None, msg


def _compare_bytes(body: bytes, expect: bytes, label: str, rtt):
    if body != expect:
        return _fail(rtt, f"body != {label} ({len(body)} vs {len(expect)} B)")
    return _ok(rtt)


def _rewrite_marker_line(text: str, prefix: str, n: int) -> str:
    """Ensure a single line `prefix N` exists (replace if present, else prepend)."""
    line = f"{prefix} {n}"
    lines = text.splitlines(keepends=True)
    out = []
    found = False
    for L in lines:
        if L.lstrip().startswith(prefix):
            nl = "\n" if L.endswith("\n") else ""
            out.append(line + nl)
            found = True
        else:
            out.append(L)
    if not found:
        out.insert(0, line + "\n")
    return "".join(out)


# Per-pass cache for recursive SD CSV listing (cleared when cache=False).
_SD_CSV_CACHE: list[tuple[str, int]] | None = None


def _list_sd_csvs(ctx, cache: bool = False) -> list[tuple[str, int]]:
    """Return [(rel_path, size), ...] for all .csv under /sd via files.json.

    cache=False: drop any prior cache, walk the tree, store result, return it.
    cache=True: return the cached list (empty if never filled this pass).
    """
    global _SD_CSV_CACHE
    if cache:
        return list(_SD_CSV_CACHE or [])

    _SD_CSV_CACHE = None
    entries: list[tuple[str, int]] = []
    dirs = [""]

    while dirs:
        d = dirs.pop(0)
        q = "/files.json"
        if d:
            q += f"?dir={urllib.parse.quote(d)}"
        st, _, body, _ = ctx.get(q, timed=False)
        if st != 200:
            _SD_CSV_CACHE = entries
            return entries
        try:
            listing = json.loads(body.decode("utf-8", errors="replace"))
        except json.JSONDecodeError:
            break
        if not isinstance(listing, list):
            break
        for item in listing:
            name = item.get("name", "")
            if item.get("dir") == 1:
                dirs.append(f"{d}/{name}" if d else name)
            elif name.lower().endswith(".csv"):
                rel = f"{d}/{name}" if d else name
                entries.append((rel.lstrip("/"), int(item.get("size", 0))))

    _SD_CSV_CACHE = entries
    return entries


# ---------------------------------------------------------------------------
# Test cases — one function each; return (result, rtt_ms|None, err)
# ---------------------------------------------------------------------------


def test_get_root(ctx: Ctx):
    st, _, body, rtt = ctx.get("/")
    if st != 200:
        return _fail(rtt, f"HTTP {st}")
    if b"rdzTTGO" not in body and b"rdzTTGOserver" not in body:
        return _fail(rtt, "missing rdzTTGO marker")
    return _ok(rtt)


def test_get_index(ctx: Ctx):
    st, _, body, rtt = ctx.get("/index.html")
    if st != 200:
        return _fail(rtt, f"HTTP {st}")
    if b"%VERSION_NAME%" in body:
        return _fail(rtt, "template not substituted")
    if b"rdzTTGO" not in body and b"rdzTTGOserver" not in body:
        return _fail(rtt, "missing rdzTTGO marker")
    return _ok(rtt)


def _make_static_compare(name: str):
    def _fn(ctx: Ctx):
        st, _, body, rtt = ctx.get("/" + name)
        if st != 200:
            return _fail(rtt, f"HTTP {st}")
        expect = (ctx.data_dir / name).read_bytes()
        return _compare_bytes(body, expect, f"RX_FSK/data/{name}", rtt)

    _fn.__name__ = f"test_static_{name.replace('.', '_')}"
    return _fn


def test_livemap_html(ctx: Ctx):
    st, _, body, rtt = ctx.get("/livemap.html")
    if st != 200:
        return _fail(rtt, f"HTTP {st}")
    if b"%MAPCENTER%" in body:
        return _fail(rtt, "MAPCENTER not substituted")
    return _ok(rtt)


def test_upd_html(ctx: Ctx):
    st, _, body, rtt = ctx.get("/upd.html")
    if st != 200:
        return _fail(rtt, f"HTTP {st}")
    if b"%FULLNAMEID%" in body or b"%LOCAL_UPDATES%" in body:
        return _fail(rtt, "template not substituted")
    return _ok(rtt)


def test_status_json(ctx: Ctx):
    st, _, body, rtt = ctx.get("/status.json")
    if st != 200:
        return _fail(rtt, f"HTTP {st}")
    try:
        obj = json.loads(body.decode("utf-8", errors="replace"))
    except json.JSONDecodeError as e:
        return _fail(rtt, f"JSON: {e}")
    if not isinstance(obj, dict) or not obj:
        return _fail(rtt, "empty or non-object")
    for k, v in obj.items():
        if not isinstance(k, str) or not isinstance(v, str):
            return _fail(rtt, f"non-string entry {k!r}")
    return _ok(rtt)


def test_status_txt(ctx: Ctx):
    st, _, body, rtt = ctx.get("/status.txt")
    if st != 200:
        return _fail(rtt, f"HTTP {st}")
    return _ok(rtt)


def test_status_html(ctx: Ctx):
    st, hdrs, _, rtt = ctx.get("/status.html")
    if st not in (301, 302, 303, 307, 308):
        return _fail(rtt, f"expected redirect, HTTP {st}")
    loc = (hdrs.get("location") or "").strip()
    if "/decoder.html" not in loc:
        return _fail(rtt, f"Location={loc!r}, want /decoder.html")
    return _ok(rtt)


def test_decoder_html(ctx: Ctx):
    st, _, body, rtt = ctx.get("/decoder.html")
    if st != 200:
        return _fail(rtt, f"HTTP {st}")
    if b"decoder.js" not in body or b"decoder-status" not in body:
        return _fail(rtt, "unexpected HTML")
    return _ok(rtt)


def test_decoder_json(ctx: Ctx):
    st, _, body, rtt = ctx.get("/decoder.json")
    if st != 200:
        return _fail(rtt, f"HTTP {st}")
    try:
        obj = json.loads(body.decode("utf-8", errors="replace"))
    except json.JSONDecodeError as e:
        return _fail(rtt, f"JSON: {e}")
    if not isinstance(obj, dict):
        return _fail(rtt, "non-object")
    for key in ("now", "full", "state", "detail", "ch", "freq", "type", "entries"):
        if key not in obj:
            return _fail(rtt, f"missing {key}")
    if obj.get("full") is not True:
        return _fail(rtt, "full snapshot expected")
    if not isinstance(obj["entries"], list):
        return _fail(rtt, "entries not a list")
    if obj["state"] not in ("disabled", "spectrum", "scan", "decode"):
        return _fail(rtt, f"bad state {obj['state']!r}")

    since = obj["now"]
    st2, _, body2, rtt2 = ctx.get(f"/decoder.json?since={since}")
    rtt = (rtt or 0) + (rtt2 or 0)
    if st2 != 200:
        return _fail(rtt, f"delta HTTP {st2}")
    try:
        delta = json.loads(body2.decode("utf-8", errors="replace"))
    except json.JSONDecodeError as e:
        return _fail(rtt, f"delta JSON: {e}")
    if delta.get("full") is True:
        # QRG reload between calls is rare; still require shape
        if "entries" not in delta:
            return _fail(rtt, "delta missing entries")
        return _ok(rtt)
    if not isinstance(delta.get("entries"), list):
        return _fail(rtt, "delta entries not a list")
    for key in ("now", "state", "detail", "ch"):
        if key not in delta:
            return _fail(rtt, f"delta missing {key}")
    return _ok(rtt)


def test_live_json(ctx: Ctx):
    st, _, body, rtt = ctx.get("/live.json")
    if st != 200:
        return _fail(rtt, f"HTTP {st}")
    try:
        obj = json.loads(body.decode("utf-8", errors="replace"))
    except json.JSONDecodeError as e:
        return _fail(rtt, f"JSON: {e}")
    if not isinstance(obj, dict) or "sonde" not in obj:
        return _fail(rtt, 'missing "sonde"')
    return _ok(rtt)


def test_qrg_json(ctx: Ctx):
    st, _, body, rtt = ctx.get("/qrg.json")
    if st != 200:
        return _fail(rtt, f"HTTP {st}")
    try:
        obj = json.loads(body.decode("utf-8", errors="replace"))
    except json.JSONDecodeError as e:
        return _fail(rtt, f"JSON: {e}")
    ch = obj.get("channels") if isinstance(obj, dict) else None
    if not isinstance(ch, list):
        return _fail(rtt, 'missing "channels"')
    for c in ch:
        for key in ("channel", "active", "freq", "launchsite", "type"):
            if key not in c:
                return _fail(rtt, f"channel missing {key}")
    return _ok(rtt)


def test_options(ctx: Ctx):
    st, _, _, rtt = ctx.request("OPTIONS", "/")
    if st != 200:
        return _fail(rtt, f"HTTP {st}")
    return _ok(rtt)


def test_file_traversal(ctx: Ctx):
    st, _, _, rtt = ctx.get("/file/../config.txt")
    if st in (200,):
        return _fail(rtt, f"unexpected HTTP {st} for traversal")
    return _ok(rtt)


def _make_lfs_file(name: str):
    def _fn(ctx: Ctx):
        st, _, body, rtt = ctx.get("/file/" + name)
        if st != 200:
            return _fail(rtt, f"HTTP {st}")
        expect = (ctx.data_dir / name).read_bytes()
        return _compare_bytes(body, expect, f"RX_FSK/data/{name}", rtt)

    _fn.__name__ = f"test_file_{name.replace('.', '_')}"
    return _fn


def test_files_json_int(ctx: Ctx):
    st, _, body, rtt = ctx.get("/files.json?dir=.int")
    if st != 200:
        return _fail(rtt, f"HTTP {st}")
    try:
        arr = json.loads(body.decode("utf-8", errors="replace"))
    except json.JSONDecodeError as e:
        return _fail(rtt, f"JSON: {e}")
    if not isinstance(arr, list):
        return _fail(rtt, "not a list")
    names = {x.get("name") for x in arr if isinstance(x, dict)}
    for need in ("index.html", "style.css", "config.txt"):
        if need not in names:
            return _fail(rtt, f"missing {need} in listing")
    return _ok(rtt)


def test_sd_list(ctx: Ctx):
    st, _, body, rtt = ctx.get("/files.json")
    if st != 200:
        return _skip(f"HTTP {st} (no SD?)")
    try:
        arr = json.loads(body.decode("utf-8", errors="replace"))
    except json.JSONDecodeError as e:
        return _fail(rtt, f"JSON: {e}")
    if not isinstance(arr, list):
        return _fail(rtt, "not a list")
    return _ok(rtt)


def test_sd_list_recursive(ctx: Ctx):
    """Walk /files.json recursively; time the whole walk; fill CSV cache."""
    st, _, _, _ = ctx.get("/files.json", timed=False)
    if st != 200:
        _list_sd_csvs(ctx, cache=False)  # clear + store empty
        return _skip(f"HTTP {st} (no SD?)")
    t0 = time.perf_counter()
    csvs = _list_sd_csvs(ctx, cache=False)
    rtt = (time.perf_counter() - t0) * 1000.0
    return _ok(rtt, f"{len(csvs)} csv")


def test_sd_csv_smallest(ctx: Ctx):
    csvs = _list_sd_csvs(ctx, cache=True)
    if not csvs:
        return _fail(None, "no csv list (empty cache)")
    rel, size = min(csvs, key=lambda x: x[1])
    st, _, body, rtt = ctx.get("/sd/" + urllib.parse.quote(rel, safe="/"))
    if st != 200:
        return _fail(rtt, f"HTTP {st} for {rel}")
    if size > 0 and len(body) == 0:
        return _fail(rtt, "empty body")
    return _ok(rtt)


def test_sd_csv_largest(ctx: Ctx):
    csvs = _list_sd_csvs(ctx, cache=True)
    if not csvs:
        return _fail(None, "no csv list (empty cache)")
    rel, size = max(csvs, key=lambda x: x[1])
    st, _, body, rtt = ctx.get("/sd/" + urllib.parse.quote(rel, safe="/"))
    if st != 200:
        return _fail(rtt, f"HTTP {st} for {rel}")
    if size > 0 and len(body) == 0:
        return _fail(rtt, "empty body")
    return _ok(rtt)


def test_login(ctx: Ctx):
    """GET /login.html → POST admin/admin; expect Set-Cookie SESSION=."""
    st, _, body, _ = ctx.get("/login.html", timed=False)
    if st == 404:
        return _skip("route not implemented")
    if st != 200:
        return _fail(None, f"GET login HTTP {st}")
    text = body.decode("utf-8", errors="replace")
    m = re.search(r'id=["\']preauth["\'][^>]*value=["\']([^"\']*)["\']', text)
    if not m:
        m = re.search(r'name=["\']preauth["\'][^>]*value=["\']([^"\']*)["\']', text)
    if not m:
        return _skip("no preauth field")
    preauth = m.group(1)
    if preauth == "%PREAUTH%" or not preauth:
        return _skip("PREAUTH not substituted (no template handler)")

    user, password = "admin", "admin"
    auth = hashlib.sha256(f"{user}:{preauth}:{password}".encode("utf-8")).hexdigest()
    st, hdr, _, rtt = ctx.post_form(
        "/login.html",
        {"user": user, "preauth": preauth, "auth": auth, "pass": ""},
        timed=True,
    )
    if st == 404:
        return _skip("POST /login.html not implemented")
    if st in (400, 401):
        return _fail(rtt, f"HTTP {st}")
    set_cookie = hdr.get("set-cookie", "")
    if "SESSION=" not in set_cookie:
        return _fail(rtt, f"no SESSION cookie (HTTP {st})")
    return _ok(rtt)


def test_config_put(ctx: Ctx):
    st, _, body, _ = ctx.get("/file/config.txt", timed=False)
    if st != 200:
        return _fail(None, f"GET config HTTP {st}")
    text = body.decode("utf-8", errors="replace")
    # bump counter if marker present
    m = re.search(r"# webserver-test (\d+)", text)
    n = int(m.group(1)) + 1 if m else 1
    new_text = _rewrite_marker_line(text, "# webserver-test", n)
    new_bytes = new_text.encode("utf-8")

    st, _, _, rtt = ctx.put("/file/config.txt", new_bytes, timed=True)
    if st not in (200, 204):
        return _fail(rtt, f"PUT HTTP {st}")

    st2, _, body2, _ = ctx.get("/file/config.txt", timed=False)
    if st2 != 200:
        return _fail(rtt, f"verify GET HTTP {st2}")
    if f"# webserver-test {n}" not in body2.decode("utf-8", errors="replace"):
        return _fail(rtt, "marker missing after PUT")
    return _ok(rtt)


def test_qrg_post(ctx: Ctx):
    st, _, _, _ = ctx.get("/qrg.html", timed=False)
    if st == 404:
        return _skip("route not implemented")
    if st != 200:
        return _fail(None, f"GET /qrg.html HTTP {st}")

    st, _, body, _ = ctx.get("/qrg.json", timed=False)
    if st != 200:
        return _fail(None, f"GET /qrg.json HTTP {st}")
    try:
        obj = json.loads(body.decode("utf-8", errors="replace"))
    except json.JSONDecodeError as e:
        return _fail(None, f"JSON: {e}")
    channels = obj.get("channels") or []
    if not channels:
        return _fail(None, "no channels")

    # pick unused (active==0) or existing webserver-test
    target = None
    for c in channels:
        if c.get("launchsite", "").strip() == "webserver-test":
            target = c
            break
    if target is None:
        for c in channels:
            if not c.get("active"):
                target = c
                break
    if target is None:
        return _skip("no unused QRG slot")

    ch = int(target["channel"])
    freq = float(target.get("freq") or 400.0)
    if target.get("launchsite", "").strip() == "webserver-test":
        freq = round(freq + 0.1, 3)
        if freq >= 406.0:
            freq = 400.0
    else:
        freq = 400.0

    # POST must send every channel (handler rewrites whole file)
    fields: dict[str, str] = {}
    for c in channels:
        i = int(c["channel"])
        site = c.get("launchsite") or ""
        f = float(c.get("freq") or 0)
        typ = (c.get("type") or "4").strip()
        # Map sondeTypeStr ("RS41","DFM",…) → form char (see sondeTypeChar)
        tu = typ.upper()
        if tu.startswith("RS41") or typ == "4":
            tch = "4"
        elif tu.startswith("RS92") or typ == "R":
            tch = "R"
        elif tu.startswith("DFM") or typ == "D":
            tch = "D"
        elif tu.startswith("MP3") or typ == "3":
            tch = "3"
        elif tu.startswith("M"):
            tch = "M"
        else:
            tch = typ[0] if typ else "4"
        active = bool(c.get("active"))
        if i == ch:
            site = "webserver-test"
            f = freq
            active = False  # keep inactive
            tch = "4"
        fields[f"F{i}"] = f"{f:.3f}"
        fields[f"T{i}"] = tch
        fields[f"S{i}"] = site
        if active:
            fields[f"A{i}"] = "on"

    st, _, _, rtt = ctx.post_form("/qrg.html", fields, timed=True)
    if st == 404:
        return _skip("POST not implemented")
    if st not in (200, 302):
        return _fail(rtt, f"POST HTTP {st}")

    st2, _, qbody, _ = ctx.get("/file/qrg.txt", timed=False)
    if st2 != 200:
        return _fail(rtt, f"verify GET qrg.txt HTTP {st2}")
    qtxt = qbody.decode("utf-8", errors="replace")
    if "webserver-test" not in qtxt:
        return _fail(rtt, "webserver-test missing in qrg.txt")
    if f"{freq:.3f}" not in qtxt:
        return _fail(rtt, f"freq {freq:.3f} missing in qrg.txt")
    return _ok(rtt)


def test_wifi_post(ctx: Ctx):
    st, _, _, _ = ctx.get("/wifi.html", timed=False)
    if st == 404:
        return _skip("route not implemented")
    if st != 200:
        return _fail(None, f"GET /wifi.html HTTP {st}")

    st, _, body, _ = ctx.get("/file/networks.txt", timed=False)
    if st != 200:
        return _fail(None, f"GET networks.txt HTTP {st}")
    lines = [L.strip() for L in body.decode("utf-8", errors="replace").splitlines() if L.strip()]
    # pairs: ssid, pw
    nets: list[tuple[str, str]] = []
    for i in range(0, len(lines) - 1, 2):
        nets.append((lines[i], lines[i + 1]))

    # find / bump webserver-test
    n = 1
    found = False
    for i, (ssid, pw) in enumerate(nets):
        if ssid == "webserver-test":
            m = re.search(r"webtest-(\d+)", pw)
            n = int(m.group(1)) + 1 if m else 1
            nets[i] = (ssid, f"webtest-{n}")
            found = True
            break
    if not found:
        if len(nets) >= 10:
            return _skip("networks full (MAX_WIFI=10)")
        nets.append(("webserver-test", f"webtest-{n}"))

    fields: dict[str, str] = {}
    for i in range(10):
        if i < len(nets):
            fields[f"S{i + 1}"] = nets[i][0]
            fields[f"P{i + 1}"] = nets[i][1]
        else:
            fields[f"S{i + 1}"] = ""
            fields[f"P{i + 1}"] = ""

    st, _, _, rtt = ctx.post_form("/wifi.html", fields, timed=True)
    if st == 404:
        return _skip("POST not implemented")
    if st not in (200, 302):
        return _fail(rtt, f"POST HTTP {st}")

    st2, _, nbody, _ = ctx.get("/file/networks.txt", timed=False)
    if st2 != 200:
        return _fail(rtt, f"verify GET HTTP {st2}")
    ntxt = nbody.decode("utf-8", errors="replace")
    if "webserver-test" not in ntxt or f"webtest-{n}" not in ntxt:
        return _fail(rtt, "webserver-test / password missing")
    return _ok(rtt)


# ---------------------------------------------------------------------------
# Case table — add/remove rows here; --only filters by name
# ---------------------------------------------------------------------------

CASES = [
    ("get /", test_get_root),
    ("get /index.html", test_get_index),
]
for _n in STATIC_COMPARE:
    CASES.append((f"get /{_n}", _make_static_compare(_n)))
CASES += [
    ("get /livemap.html", test_livemap_html),
    ("get /upd.html", test_upd_html),
    ("get /status.json", test_status_json),
    ("get /status.txt", test_status_txt),
    ("get /status.html", test_status_html),
    ("get /decoder.html", test_decoder_html),
    ("get /decoder.json", test_decoder_json),
    ("get /live.json", test_live_json),
    ("get /qrg.json", test_qrg_json),
    ("OPTIONS /", test_options),
    ("GET /file/../…", test_file_traversal),
]
for _n in LFS_COMPARE:
    if _n in MUTATED:
        continue
    CASES.append((f"get /file/{_n}", _make_lfs_file(_n)))
CASES += [
    ("get /files.json?dir=.int", test_files_json_int),
    ("get /files.json (SD)", test_sd_list),
    ("SD list recursive", test_sd_list_recursive),
    ("get /sd smallest.csv", test_sd_csv_smallest),
    ("get /sd largest.csv", test_sd_csv_largest),
    ("login admin/admin", test_login),
    ("PUT /file/config.txt", test_config_put),
    ("POST /qrg.html", test_qrg_post),
    ("POST /wifi.html", test_wifi_post),
]


# ---------------------------------------------------------------------------
# Controller + reporting
# ---------------------------------------------------------------------------


class Stats:
    __slots__ = ("ok", "fail", "skip", "rtt_sum", "rtt_n", "rtt_min", "rtt_max", "last_err", "last")

    def __init__(self):
        self.ok = self.fail = self.skip = 0
        self.rtt_sum = 0.0
        self.rtt_n = 0
        self.rtt_min: float | None = None
        self.rtt_max: float | None = None
        self.last_err = ""
        self.last = ""

    def update(self, result: str, rtt: float | None, err: str):
        self.last = result
        if result == "ok":
            self.ok += 1
        elif result == "fail":
            self.fail += 1
            if err:
                self.last_err = err
        else:
            self.skip += 1
            if err and not self.last_err:
                self.last_err = err
        if rtt is not None:
            self.rtt_sum += rtt
            self.rtt_n += 1
            self.rtt_min = rtt if self.rtt_min is None else min(self.rtt_min, rtt)
            self.rtt_max = rtt if self.rtt_max is None else max(self.rtt_max, rtt)


def _fmt_rtt(rtt: float | None) -> str:
    if rtt is None:
        return "-"
    return f"{rtt:.0f}ms"


def _print_header(loop: bool):
    if loop:
        print(
            f"{'name':<28} {'result':<6} {'rtt':>7} {'min':>7} {'max':>7} {'avg':>7} {'ok':>4} {'err':>4}  last error"
        )
    else:
        print(f"{'name':<28} {'result':<6} {'rtt':>7}  error")


def _print_line(name: str, result: str, rtt: float | None, err: str, st: Stats | None, loop: bool):
    if loop and st is not None:
        avg = f"{st.rtt_sum / st.rtt_n:.0f}ms" if st.rtt_n else "-"
        mn = _fmt_rtt(st.rtt_min)
        mx = _fmt_rtt(st.rtt_max)
        print(
            f"{name:<28} {result:<6} {_fmt_rtt(rtt):>7} {mn:>7} {mx:>7} {avg:>7} "
            f"{st.ok:>4} {st.fail:>4}  {st.last_err}"
        )
    else:
        print(f"{name:<28} {result:<6} {_fmt_rtt(rtt):>7}  {err}")


def run_once(
    host: str,
    port: int,
    data_dir: Path,
    cases: list,
    stats: dict[str, Stats] | None,
    loop: bool,
) -> int:
    fails = 0
    for name, fn in cases:
        ctx = Ctx(host, port, data_dir)
        try:
            result, rtt, err = fn(ctx)
        except Exception as e:
            result, rtt, err = "fail", None, str(e)
        finally:
            ctx.close()
        if result == "fail":
            fails += 1
        if stats is not None:
            st = stats.setdefault(name, Stats())
            st.update(result, rtt, err)
            _print_line(name, result, rtt, err, st, loop)
        else:
            _print_line(name, result, rtt, err, None, False)
    return fails


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="ESP32 web-interface test harness")
    ap.add_argument("--host", required=True, help="device IP or hostname")
    ap.add_argument("--port", type=int, default=80)
    ap.add_argument("--data-dir", type=Path, default=DEFAULT_DATA)
    ap.add_argument("--loop", action="store_true", help="repeat forever (or --count)")
    ap.add_argument("--interval", type=float, default=30.0, help="seconds between loop passes")
    ap.add_argument("--count", type=int, default=0, help="loop pass limit (0=forever with --loop)")
    ap.add_argument(
        "--only",
        default="",
        help="comma-separated case name substrings to run (match if any substring in name)",
    )
    args = ap.parse_args(argv)

    if not args.data_dir.is_dir():
        print(f"data-dir not found: {args.data_dir}", file=sys.stderr)
        return 2

    cases = CASES
    if args.only.strip():
        keys = [k.strip() for k in args.only.split(",") if k.strip()]
        cases = [(n, f) for n, f in CASES if any(k in n for k in keys)]
        if not cases:
            print("no cases match --only", file=sys.stderr)
            return 2

    loop = args.loop or args.count > 0
    stats: dict[str, Stats] | None = {n: Stats() for n, _ in cases} if loop else None
    total_fails = 0
    pass_n = 0

    _print_header(loop)
    try:
        while True:
            pass_n += 1
            if loop and pass_n > 1:
                print(f"--- pass {pass_n} ---")
            fails = run_once(args.host, args.port, args.data_dir, cases, stats, loop)
            total_fails += fails
            if not loop:
                break
            if args.count and pass_n >= args.count:
                break
            time.sleep(args.interval)
    except KeyboardInterrupt:
        print("\n(interrupted)")

    return 1 if total_fails else 0


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""
Minimal SondeHub-compatible HTTP server for testing conn-sondehub and ShFreqImport.

Usage:
  python3 sondehub_test_server.py [--port PORT]

  Default port is 80 (device connects to port 80 only). Use --port 8080 to run
  without root; then test with curl. For device testing, run on 80, e.g.:
    sudo python3 sondehub_test_server.py

  On the device, set sondehub.host to this machine's IP. Enable sondehub.fiactive
  to test frequency import (GET /sondes); the server returns one RS41 at 405.0 MHz.
"""

import argparse
import json
import socket
from http.server import BaseHTTPRequestHandler, HTTPServer

# Test data: one RS41 at 405.0 MHz. Add more dicts to simulate multiple sondes.
SONDE_ENTRIES = [
    {"id": "RS41-TEST", "type": "RS41", "lat": 0, "lon": 0, "frequency": 405.0},
]


def read_chunked(rfile):
    """Read body when Transfer-Encoding: chunked. Consumes until 0-sized chunk."""
    while True:
        line = rfile.readline()
        if not line:
            break
        line = line.strip().decode("latin-1")
        try:
            size = int(line.split(";")[0].strip(), 16)
        except ValueError:
            break
        if size == 0:
            break
        rfile.read(size)
        trail = rfile.readline()  # \r\n after chunk
        if not trail:
            break


class SondehubTestHandler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        print("[%s] %s" % (self.log_date_time_string(), format % args))

    def send_ok(self):
        body = b"ok"
        self.send_response(200)
        self.send_header("Content-Type", "text/plain")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def send_json(self, data):
        body = json.dumps(data).encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_PUT(self):
        path = self.path.split("?")[0]
        if path == "/listeners":
            cl = self.headers.get("Content-Length")
            if cl:
                self.rfile.read(int(cl))
            self.send_ok()
            return
        if path == "/sondes/telemetry":
            if self.headers.get("Transfer-Encoding", "").lower() == "chunked":
                read_chunked(self.rfile)
            else:
                cl = self.headers.get("Content-Length")
                if cl:
                    self.rfile.read(int(cl))
            self.send_ok()
            return
        # Unknown PUT: consume body if present, then ok
        cl = self.headers.get("Content-Length")
        if cl:
            self.rfile.read(int(cl))
        self.send_ok()

    def do_GET(self):
        path = self.path.split("?")[0]
        if path == "/sondes":
            # Build JSON in the shape ShFreqImport expects: {"id": {"lat", "lon", "tx_frequency", "type"}}
            out = {}
            for e in SONDE_ENTRIES:
                out[e["id"]] = {
                    "lat": e["lat"],
                    "lon": e["lon"],
                    "tx_frequency": e["frequency"],
                    "type": e["type"],
                }
            self.send_json(out)
            return
        self.send_ok()


def get_local_ip():
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(("8.8.8.8", 80))
            return s.getsockname()[0]
    except Exception:
        return "127.0.0.1"


def main():
    parser = argparse.ArgumentParser(
        description="SondeHub-compatible test server for conn-sondehub and ShFreqImport."
    )
    parser.add_argument(
        "--port",
        type=int,
        default=80,
        help="Port to listen on (default: 80; device only connects to 80)",
    )
    args = parser.parse_args()
    port = args.port
    server = HTTPServer(("", port), SondehubTestHandler)
    ip = get_local_ip()
    print("SondeHub test server listening on %s:%d" % (ip, port))
    print("Configure sondehub.host to %s (and run on port 80 for device testing)" % ip)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nShutting down")
        server.shutdown()


if __name__ == "__main__":
    main()

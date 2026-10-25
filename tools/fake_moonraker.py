#!/usr/bin/env python3
# A stand-in Moonraker for AFC parity runs in batch mode: AFC connects to
# Moonraker at PREP and only then creates its stats (afc_stats), which every
# TOOL_LOAD / TOOL_UNLOAD updates. Without it both the python and the native
# AFC stop at the first one (an AttributeError in the python AFC).
# Answers what AFC_utils' client asks, stateless so every run sees the same:
# server/info, server/config (no spoolman, td1 or lane_data), an empty
# database namespace, and success for every write.
#   fake_moonraker.py PORT
import json
import sys
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse


class Handler(BaseHTTPRequestHandler):
    def reply(self, result, code=200):
        body = json.dumps({"result": result}).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        url = urlparse(self.path)
        if url.path == "/server/info":
            self.reply({"klippy_state": "ready"})
        elif url.path == "/server/config":
            self.reply({"orig": {}, "config": {}})
        elif url.path == "/server/database/item":
            namespace = parse_qs(url.query).get("namespace", [""])[0]
            self.reply({"namespace": namespace, "key": None, "value": {}})
        else:
            self.reply(None, 404)

    def write(self):
        self.rfile.read(int(self.headers.get("Content-Length") or 0))
        self.reply({})

    do_POST = do_DELETE = write

    def log_message(self, *args):
        pass


if __name__ == "__main__":
    ThreadingHTTPServer(("127.0.0.1", int(sys.argv[1])), Handler).serve_forever()

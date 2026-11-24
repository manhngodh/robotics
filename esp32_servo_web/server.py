#!/usr/bin/env python3
"""Local web server to control ESP32-S3 + PCA9685 16-channel servo driver via USB Serial."""

import json
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import serial
import serial.tools.list_ports

PORT_ENV = os.environ.get("ESP32_PORT", "/dev/cu.usbmodem5D300203121")
HTTP_PORT = int(os.environ.get("PORT", "8765"))
BASE_DIR = os.path.dirname(os.path.abspath(__file__))

serial_lock = threading.Lock()
ser_conn = None
current_port = PORT_ENV
angles = [90] * 16
enabled = [False] * 16


def find_esp_port() -> str:
    if os.path.exists(PORT_ENV):
        return PORT_ENV
    for p in serial.tools.list_ports.comports():
        if "usbmodem" in p.device or "usbserial" in p.device:
            return p.device
    return PORT_ENV


def get_serial():
    global ser_conn, current_port
    if ser_conn is not None and ser_conn.is_open:
        return ser_conn
    current_port = find_esp_port()
    ser_conn = serial.Serial(current_port, 115200, timeout=0.6)
    ser_conn.setDTR(False)
    ser_conn.setRTS(False)
    time.sleep(0.15)
    ser_conn.reset_input_buffer()
    return ser_conn


def send_cmd(cmd: str) -> str:
    global ser_conn
    with serial_lock:
        try:
            s = get_serial()
            s.reset_input_buffer()
            s.write((cmd.strip() + "\n").encode("utf-8"))
            s.flush()
            resp = s.readline().decode("utf-8", errors="replace").strip()
            return resp or "NO_REPLY"
        except Exception as exc:
            if ser_conn is not None:
                try:
                    ser_conn.close()
                except Exception:
                    pass
                ser_conn = None
            raise exc


class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        pass

    def _send_json(self, code: int, payload: dict):
        body = json.dumps(payload).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path == "/" or self.path == "/index.html":
            html_path = os.path.join(BASE_DIR, "index.html")
            with open(html_path, "rb") as f:
                data = f.read()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)
            return

        if self.path == "/api/status":
            try:
                reply = send_cmd("PING")
                self._send_json(200, {
                    "ok": True,
                    "port": current_port,
                    "reply": reply,
                    "angles": angles,
                    "enabled": enabled,
                })
            except Exception as exc:
                self._send_json(200, {
                    "ok": False,
                    "port": current_port,
                    "error": str(exc),
                    "angles": angles,
                    "enabled": enabled,
                })
            return

        self._send_json(404, {"error": "Not found"})

    def do_POST(self):
        length = int(self.headers.get("Content-Length", "0"))
        raw = self.rfile.read(length) if length > 0 else b"{}"
        try:
            data = json.loads(raw.decode("utf-8"))
        except Exception:
            self._send_json(400, {"ok": False, "error": "Invalid JSON"})
            return

        try:
            if self.path == "/api/servo":
                ch = int(data["ch"])
                angle = max(0, min(180, int(data["angle"])))
                reply = send_cmd(f"S {ch} {angle}")
                if 0 <= ch < 16:
                    angles[ch] = angle
                    enabled[ch] = True
                self._send_json(200, {"ok": True, "reply": reply, "ch": ch, "angle": angle})
                return

            if self.path == "/api/all":
                angle = max(0, min(180, int(data["angle"])))
                reply = send_cmd(f"A {angle}")
                for i in range(16):
                    angles[i] = angle
                    enabled[i] = True
                self._send_json(200, {"ok": True, "reply": reply, "angle": angle})
                return

            if self.path == "/api/off":
                ch = int(data.get("ch", -1))
                if ch < 0:
                    reply = send_cmd("OFF")
                    for i in range(16):
                        enabled[i] = False
                else:
                    reply = send_cmd(f"OFF {ch}")
                    if 0 <= ch < 16:
                        enabled[ch] = False
                self._send_json(200, {"ok": True, "reply": reply, "ch": ch})
                return

            self._send_json(404, {"ok": False, "error": "Unknown endpoint"})
        except Exception as exc:
            self._send_json(500, {"ok": False, "error": str(exc)})


if __name__ == "__main__":
    server = ThreadingHTTPServer(("127.0.0.1", HTTP_PORT), Handler)
    print(f"ESP32 Servo Web UI running at http://127.0.0.1:{HTTP_PORT} (port={PORT_ENV})", flush=True)
    server.serve_forever()

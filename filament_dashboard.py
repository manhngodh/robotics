#!/usr/bin/env python3
"""Real-time Filament Flow Dashboard & Software Gap Detector for Creality SPARKX i7."""

from __future__ import annotations

import argparse
import bisect
import json
import math
import os
import re
import threading
import time
import urllib.parse
import urllib.request
from collections import deque
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

PRINTER_URL = "http://192.168.100.106:7125"
LOCAL_GCODE_DIR = "/Users/manhngodh/development/robotics/slicing_output"
FILAMENT_AREA_MM2 = math.pi * (1.75 / 2.0) ** 2
G_PER_MM_DEFAULT = 46.69 / 15283.41  # updated dynamically per G-code header if present

EMPTY_INFO_RE = re.compile(
    r"^\[INFO\]\s+(\d{4}-\d{2}-\d{2}\s+\d{2}:\d{2}:\d{2},\d+)\s+.*?EmptyInfo set_old_position:\s+"
    r"old_pox_x=([-0-9.]+),\s+old_pox_y=([-0-9.]+),\s+old_pox_z=([-0-9.]+),\s+"
    r"old_pox_e=([-0-9.]+),\s+old_measuring_wheel=([-0-9.]+)",
    re.MULTILINE,
)
E_MOVE_RE = re.compile(r"\sE([-0-9.]+)")
X_MOVE_RE = re.compile(r"\sX([-0-9.]+)")
Y_MOVE_RE = re.compile(r"\sY([-0-9.]+)")
R_M73_RE = re.compile(r"R(\d+)")


class GCodeIndex:
    """Fast byte-offset and future-time index for an active OrcaSlicer M83 G-code file."""

    def __init__(self, filename: str, raw_bytes: bytes) -> None:
        self.filename = filename
        self.raw_bytes = raw_bytes
        self.offsets: list[int] = []
        self.cum_e: list[float] = []
        self.layer_offsets: dict[int, int] = {}
        self.layer_z: dict[int, float] = {}
        self.m73_points: list[tuple[int, int, int, float]] = []  # (offset, r_min, layer, e_cum)
        self.total_e_mm = 0.0
        self.g_per_mm = G_PER_MM_DEFAULT
        self._build()

    def _build(self) -> None:
        e_cum = 0.0
        cur_layer = 0
        offset = 0
        header_g = None
        header_mm = None

        for raw in self.raw_bytes.splitlines(keepends=True):
            line_len = len(raw)
            s = raw.decode("utf-8", errors="ignore").strip()
            if s.startswith("; filament used [g] ="):
                vals = [float(v.strip()) for v in s.split("=")[1].split(",") if v.strip()]
                if vals:
                    header_g = max(vals)
            elif s.startswith("; filament used [mm] ="):
                vals = [float(v.strip()) for v in s.split("=")[1].split(",") if v.strip()]
                if vals:
                    header_mm = max(vals)
            elif s.startswith(";LAYER_CHANGE"):
                cur_layer += 1
                self.layer_offsets[cur_layer] = offset
            elif s.startswith(";Z:") and cur_layer in self.layer_offsets:
                try:
                    self.layer_z[cur_layer] = float(s[3:].strip())
                except ValueError:
                    pass
            elif s.startswith("M73 "):
                m_r = R_M73_RE.search(s)
                if m_r:
                    self.m73_points.append((offset, int(m_r.group(1)), cur_layer, e_cum))
            elif s.startswith(("G0 ", "G1 ", "G2 ", "G3 ")) and " E" in s:
                m = E_MOVE_RE.search(s)
                if m:
                    e_cum += float(m.group(1))
                    self.offsets.append(offset)
                    self.cum_e.append(e_cum)
            offset += line_len

        self.total_e_mm = e_cum
        if header_g and header_mm and header_mm > 0:
            self.g_per_mm = header_g / header_mm

    def e_at_offset(self, pos: int) -> float:
        idx = bisect.bisect_right(self.offsets, pos) - 1
        return self.cum_e[idx] if idx >= 0 else 0.0

    def forecast(self, pos: int) -> dict:
        e_now = self.e_at_offset(pos)
        rem_mm = max(0.0, self.total_e_mm - e_now)
        future_pts = [p for p in self.m73_points if p[0] >= pos]
        r_now = future_pts[0][1] if future_pts else 0
        horizons = []
        for dt in [15, 30, 60, 120]:
            if r_now <= 0:
                break
            if dt >= r_now:
                continue
            target_r = r_now - dt
            match = next((p for p in future_pts if p[1] <= target_r), None)
            if match:
                de = max(0.0, match[3] - e_now)
                horizons.append(
                    {
                        "label": f"+{dt} phút",
                        "target_layer": match[2],
                        "add_m": round(de / 1000.0, 2),
                        "add_g": round(de * self.g_per_mm, 2),
                        "cum_m": round(match[3] / 1000.0, 2),
                    }
                )
        horizons.append(
            {
                "label": f"Hoàn thành (~{r_now}p)",
                "target_layer": max(self.layer_offsets.keys()) if self.layer_offsets else 0,
                "add_m": round(rem_mm / 1000.0, 2),
                "add_g": round(rem_mm * self.g_per_mm, 2),
                "cum_m": round(self.total_e_mm / 1000.0, 2),
            }
        )
        return {
            "extruded_m": round(e_now / 1000.0, 2),
            "extruded_g": round(e_now * self.g_per_mm, 1),
            "total_m": round(self.total_e_mm / 1000.0, 2),
            "total_g": round(self.total_e_mm * self.g_per_mm, 1),
            "remaining_m": round(rem_mm / 1000.0, 2),
            "remaining_g": round(rem_mm * self.g_per_mm, 1),
            "est_remaining_min": r_now,
            "horizons": horizons,
        }

    def analyze_segment(self, start_byte: int, end_byte: int) -> dict:
        chunk = self.raw_bytes[max(0, start_byte) : max(0, end_byte)].decode("utf-8", errors="ignore")
        xs: list[float] = []
        ys: list[float] = []
        e_sum = 0.0
        move_count = 0
        for line in chunk.splitlines():
            s = line.strip()
            if s.startswith(("G0 ", "G1 ")) and " E" in s:
                me = E_MOVE_RE.search(s)
                if me and float(me.group(1)) > 0:
                    e_sum += float(me.group(1))
                    move_count += 1
                    mx = X_MOVE_RE.search(s)
                    my = Y_MOVE_RE.search(s)
                    if mx:
                        xs.append(float(mx.group(1)))
                    if my:
                        ys.append(float(my.group(1)))
        return {
            "moves": move_count,
            "gcode_e_mm": round(e_sum, 2),
            "bbox": (
                [round(min(xs), 1), round(min(ys), 1), round(max(xs), 1), round(max(ys), 1)]
                if xs and ys
                else None
            ),
        }

    def build_recovery_gcode(self, mode: str, layer: int, start_byte: int = 0, end_byte: int = 0, bed_temp: int = 70, noz_temp: int = 265) -> str:
        z_target = self.layer_z.get(layer)
        if z_target is None and self.layer_z:
            nearest = min(self.layer_z.keys(), key=lambda k: abs(k - layer))
            z_target = self.layer_z[nearest]
            layer = nearest
        z_safe = round((z_target or 10.0) + 8.0, 2)

        header = [
            f"; === SAFE RECOVERY GCODE ({mode.upper()}) ===",
            f"; Source: {self.filename} | Target Layer {layer} (Z={z_target})",
            "; IMPORTANT: NO START_PRINT / NO G28 Z! Keep bed heated & axes homed (or use G28 X Y + SET_KINEMATIC_POSITION).",
            f"M140 S{bed_temp}",
            f"M109 S{noz_temp}",
            "G90",
            "M83 ; relative extrusion",
            "G92 E0",
            f"G1 Z{z_safe} F1200 ; safe Z clearance above printed part",
            "; Optional small purge if parked at waste chute X277 Y130:",
            "; G1 E15 F240",
            "; G92 E0",
            "",
        ]

        if mode == "gap" and end_byte > start_byte >= 0:
            body_bytes = self.raw_bytes[start_byte:end_byte]
            body = body_bytes.decode("utf-8", errors="ignore")
            footer = [
                "",
                f"G1 Z{z_safe} F1200 ; lift safely after gap patch",
            ]
            return "\n".join(header) + body + "\n" + "\n".join(footer) + "\n"

        # Default: resume from layer start to end of file
        layer_off = self.layer_offsets.get(layer, 0)
        body = self.raw_bytes[layer_off:].decode("utf-8", errors="ignore")
        return "\n".join(header) + body


class PrinterMonitor:
    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.flow_history: deque[dict] = deque(maxlen=150)  # ~2.5 min @ 1s
        self.wheel_history: deque[dict] = deque(maxlen=60)  # ~6 min @ 6s
        self.gap_events: deque[dict] = deque(maxlen=30)
        self.gcode_index: GCodeIndex | None = None
        self.last_state: dict = {"connected": False, "error": "Initializing..."}
        self.last_empty_ts: str | None = None
        self.last_empty_entry: tuple[str, float, float, float, float, float, float] | None = None
        self.last_pos_at_empty: dict[str, tuple[int, int, float]] = {}
        self.auto_pause_enabled = False
        self.consecutive_gaps = 0
        self.auto_pause_triggered_at: str | None = None

    def _fetch_json(self, path: str) -> dict:
        with urllib.request.urlopen(f"{PRINTER_URL}{path}", timeout=3.0) as resp:
            return json.load(resp)

    def _post_gcode(self, script: str) -> dict:
        encoded = urllib.parse.quote(script)
        req = urllib.request.Request(f"{PRINTER_URL}/printer/gcode/script?script={encoded}", method="POST")
        with urllib.request.urlopen(req, timeout=5.0) as resp:
            return json.load(resp)

    def _ensure_gcode_index(self, filename: str) -> None:
        if not filename:
            return
        if self.gcode_index and self.gcode_index.filename == filename:
            return
        local_path = os.path.join(LOCAL_GCODE_DIR, filename)
        raw: bytes | None = None
        if os.path.isfile(local_path):
            with open(local_path, "rb") as f:
                raw = f.read()
        else:
            url = f"{PRINTER_URL}/server/files/gcodes/{urllib.parse.quote(filename)}"
            with urllib.request.urlopen(url, timeout=15.0) as resp:
                raw = resp.read()
        if raw:
            self.gcode_index = GCodeIndex(filename, raw)

    def _fetch_empty_info_tail(self, tail_bytes: int = 49152) -> list[tuple[str, float, float, float, float, float]]:
        req = urllib.request.Request(
            f"{PRINTER_URL}/server/files/logs/klippy.log",
            headers={"Range": f"bytes=-{tail_bytes}"},
        )
        with urllib.request.urlopen(req, timeout=3.0) as resp:
            text = resp.read().decode("utf-8", errors="replace")
        out = []
        for m in EMPTY_INFO_RE.finditer(text):
            out.append(
                (
                    m.group(1),
                    float(m.group(2)),
                    float(m.group(3)),
                    float(m.group(4)),
                    float(m.group(5)),
                    float(m.group(6)),
                )
            )
        return out

    def _classify_interval(self, d_e: float, d_w: float) -> tuple[str, float | None]:
        if d_e <= 0.4:
            return "IDLE", None
        ratio = d_w / d_e
        if d_e <= 2.5:
            return "SHORT", round(ratio, 3)
        if ratio < 0.20:
            return "GAP", round(ratio, 3)
        if ratio < 0.45:
            return "WARN", round(ratio, 3)
        return "OK", round(ratio, 3)

    def poll_once(self) -> None:
        try:
            q = self._fetch_json(
                "/printer/objects/query?print_stats&virtual_sdcard&extruder&heater_bed&motion_report&filament_inventory_manager&box"
            )["result"]["status"]
            vsd = q["virtual_sdcard"]
            ext = q["extruder"]
            bed = q["heater_bed"]
            mr = q["motion_report"]
            ps = q["print_stats"]
            fim = q.get("filament_inventory_manager", {})
            box = q.get("box", {})

            filename = vsd.get("file_name") or ps.get("filename") or ""
            self._ensure_gcode_index(filename)

            cur_byte = int(vsd.get("file_position") or 0)
            cur_layer = int(vsd.get("layer") or 0)
            layer_count = int(vsd.get("layer_count") or 0)
            z_live = float(mr["live_position"][2])
            e_vel = float(mr["live_extruder_velocity"])
            flow = round(e_vel * FILAMENT_AREA_MM2, 2)
            now_str = datetime.now().strftime("%H:%M:%S")

            entries = self._fetch_empty_info_tail()
            with self.lock:
                self.flow_history.append(
                    {
                        "t": now_str,
                        "e_vel": round(e_vel, 2),
                        "flow": flow,
                        "z": round(z_live, 2),
                        "layer": cur_layer,
                    }
                )

                # Seed / process new EmptyInfo entries
                for idx, entry in enumerate(entries):
                    ts, x, y, z, e_pos, wheel = entry
                    if ts not in self.last_pos_at_empty:
                        self.last_pos_at_empty[ts] = (cur_byte, cur_layer, z_live)

                    if self.last_empty_ts is None:
                        # Bootstrap previous interval cleanly on startup
                        if idx > 0:
                            p_ts, _px, _py, _pz, p_e, p_w = entries[idx - 1]
                            d_e = e_pos - p_e
                            d_w = abs(wheel - p_w)
                            if 0 <= d_e < 200:
                                status_tag, ratio = self._classify_interval(d_e, d_w)
                                self.wheel_history.append(
                                    {
                                        "t": ts.split()[1][:8],
                                        "ts_full": ts,
                                        "z": round(z, 2),
                                        "d_e": round(d_e, 2),
                                        "d_w": round(d_w, 2),
                                        "ratio": ratio,
                                        "status": status_tag,
                                        "start_byte": cur_byte,
                                        "end_byte": cur_byte,
                                        "layer": cur_layer,
                                    }
                                )
                        if idx == len(entries) - 1:
                            self.last_empty_ts = ts
                            self.last_empty_entry = (*entry, cur_byte)
                    elif ts > self.last_empty_ts and self.last_empty_entry is not None:
                        p_ts, _px, _py, _pz, p_e, p_w, p_byte = self.last_empty_entry
                        d_e = e_pos - p_e
                        d_w = abs(wheel - p_w)
                        if 0 <= d_e < 200:
                            status_tag, ratio = self._classify_interval(d_e, d_w)
                            item = {
                                "t": ts.split()[1][:8],
                                "ts_full": ts,
                                "z": round(z, 2),
                                "d_e": round(d_e, 2),
                                "d_w": round(d_w, 2),
                                "ratio": ratio,
                                "status": status_tag,
                                "start_byte": p_byte,
                                "end_byte": cur_byte,
                                "layer": cur_layer,
                            }
                            self.wheel_history.append(item)
                            if status_tag == "GAP":
                                self.consecutive_gaps += 1
                                seg = (
                                    self.gcode_index.analyze_segment(p_byte, cur_byte)
                                    if self.gcode_index and cur_byte > p_byte
                                    else {"moves": 0, "gcode_e_mm": round(d_e, 2), "bbox": None}
                                )
                                gap_ev = {
                                    "t": item["t"],
                                    "layer": cur_layer,
                                    "z": round(z, 2),
                                    "d_e": round(d_e, 2),
                                    "d_w": round(d_w, 2),
                                    "missing_mm": round(max(0.0, d_e - d_w), 2),
                                    "ratio": ratio,
                                    "start_byte": p_byte,
                                    "end_byte": cur_byte,
                                    "bbox": seg["bbox"],
                                    "moves": seg["moves"],
                                }
                                self.gap_events.append(gap_ev)
                                if (
                                    self.auto_pause_enabled
                                    and self.consecutive_gaps >= 2
                                    and vsd.get("is_active")
                                    and ps.get("state") != "paused"
                                ):
                                    try:
                                        self._post_gcode("PAUSE")
                                        self.auto_pause_triggered_at = item["t"]
                                    except Exception:
                                        pass
                            else:
                                self.consecutive_gaps = 0

                        self.last_empty_ts = ts
                        self.last_empty_entry = (*entry, cur_byte)

                forecast = self.gcode_index.forecast(cur_byte) if self.gcode_index else None
                t1_info = box.get("T1", {})
                latest_w = self.wheel_history[-1] if self.wheel_history else None
                overall_state = "OK"
                if self.consecutive_gaps >= 2:
                    overall_state = "CRITICAL_GAP"
                elif latest_w and latest_w["status"] in ("GAP", "WARN"):
                    overall_state = latest_w["status"]

                self.last_state = {
                    "connected": True,
                    "updated_at": now_str,
                    "filename": filename,
                    "print_active": bool(vsd.get("is_active")),
                    "print_state": ps.get("state"),
                    "progress_pct": round(float(vsd.get("progress") or 0) * 100.0, 1),
                    "layer": cur_layer,
                    "layer_count": layer_count,
                    "z_mm": round(z_live, 2),
                    "file_position": cur_byte,
                    "file_size": int(vsd.get("file_size") or 0),
                    "noz_temp": round(float(ext["temperature"]), 1),
                    "noz_target": round(float(ext["target"]), 0),
                    "bed_temp": round(float(bed["temperature"]), 1),
                    "bed_target": round(float(bed["target"]), 0),
                    "e_vel": round(e_vel, 2),
                    "flow": flow,
                    "active_slot": fim.get("active_tnn") or f"T1{t1_info.get('filament', '')}",
                    "overall_state": overall_state,
                    "auto_pause_enabled": self.auto_pause_enabled,
                    "auto_pause_triggered_at": self.auto_pause_triggered_at,
                    "latest_wheel": latest_w,
                    "forecast": forecast,
                    "flow_history": list(self.flow_history),
                    "wheel_history": list(self.wheel_history),
                    "gap_events": list(self.gap_events),
                }
        except Exception as exc:
            with self.lock:
                self.last_state = {"connected": False, "error": str(exc), "updated_at": datetime.now().strftime("%H:%M:%S")}


MONITOR = PrinterMonitor()

HTML_PAGE = """<!doctype html>
<html lang="vi">
<head>
<meta charset="utf-8" />
<meta name="viewport" content="width=device-width,initial-scale=1" />
<title>SPARKX i7 — Live Filament Flow & Gap Detector</title>
<style>
  :root {
    --bg: #0b101b; --card: #121a2b; --border: #22304c; --text: #e6edf7; --muted: #8ea0bf;
    --accent: #38bdf8; --ok: #22c55e; --warn: #f59e0b; --bad: #ef4444; --purple: #a78bfa;
  }
  * { box-sizing: border-box; }
  body { margin: 0; font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif;
    background: var(--bg); color: var(--text); padding: 18px; }
  .wrap { max-width: 1260px; margin: 0 auto; display: flex; flex-direction: column; gap: 14px; }
  .header { display: flex; flex-wrap: wrap; justify-content: space-between; align-items: center; gap: 12px;
    background: var(--card); border: 1px solid var(--border); border-radius: 12px; padding: 14px 18px; }
  .title { font-size: 18px; font-weight: 700; display: flex; align-items: center; gap: 10px; }
  .sub { font-size: 13px; color: var(--muted); margin-top: 3px; }
  .pill { padding: 4px 10px; border-radius: 999px; font-size: 12px; font-weight: 700; letter-spacing: 0.3px; }
  .pill-OK { background: rgba(34,197,94,0.16); color: var(--ok); border: 1px solid rgba(34,197,94,0.35); }
  .pill-WARN, .pill-SHORT { background: rgba(245,158,11,0.16); color: var(--warn); border: 1px solid rgba(245,158,11,0.35); }
  .pill-GAP, .pill-CRITICAL_GAP { background: rgba(239,68,68,0.18); color: var(--bad); border: 1px solid rgba(239,68,68,0.45); }
  .grid-4 { display: grid; grid-template-columns: repeat(4, minmax(0, 1fr)); gap: 12px; }
  .grid-2 { display: grid; grid-template-columns: repeat(2, minmax(0, 1fr)); gap: 12px; }
  @media (max-width: 900px) { .grid-4, .grid-2 { grid-template-columns: 1fr; } }
  .card { background: var(--card); border: 1px solid var(--border); border-radius: 12px; padding: 14px 16px; }
  .kpi-label { font-size: 12px; color: var(--muted); text-transform: uppercase; letter-spacing: 0.4px; }
  .kpi-val { font-size: 26px; font-weight: 700; margin-top: 6px; font-variant-numeric: tabular-nums; }
  .kpi-sub { font-size: 12px; color: var(--muted); margin-top: 4px; font-variant-numeric: tabular-nums; }
  h3 { margin: 0 0 10px 0; font-size: 14px; color: #cbd5e1; display: flex; justify-content: space-between; align-items: center; }
  canvas { width: 100%; height: 210px; display: block; background: #0d1423; border-radius: 8px; border: 1px solid #1a253d; }
  table { width: 100%; border-collapse: collapse; font-size: 13px; font-variant-numeric: tabular-nums; }
  th, td { text-align: left; padding: 7px 8px; border-bottom: 1px solid #1c2740; }
  th { color: var(--muted); font-weight: 600; font-size: 12px; }
  .btn { cursor: pointer; border: 1px solid var(--border); background: #1a253d; color: var(--text);
    padding: 7px 12px; border-radius: 8px; font-size: 12px; font-weight: 600; }
  .btn:hover { background: #243352; }
  .btn-warn { border-color: rgba(245,158,11,0.5); color: #fcd34d; }
  .btn-bad { border-color: rgba(239,68,68,0.55); color: #fca5a5; }
  .btn-accent { border-color: rgba(56,189,248,0.5); color: #7dd3fc; }
  .controls { display: flex; flex-wrap: wrap; gap: 8px; align-items: center; }
  .row-actions { display: flex; gap: 8px; align-items: center; flex-wrap: wrap; }
  input[type="number"] { width: 84px; background: #0d1423; border: 1px solid var(--border); color: var(--text);
    padding: 6px 8px; border-radius: 6px; font-size: 13px; }
  .switch { display: inline-flex; align-items: center; gap: 6px; font-size: 12px; color: var(--muted); cursor: pointer; }
  .mono { font-family: ui-monospace, SFMono-Regular, Menlo, monospace; }
</style>
</head>
<body>
<div class="wrap">
  <div class="header">
    <div>
      <div class="title">
        <span>Creality SPARKX i7 — Live Filament Flow &amp; Gap Detector</span>
        <span id="statusBadge" class="pill pill-OK">CONNECTING</span>
      </div>
      <div class="sub" id="fileInfo">Đang kết nối tới 192.168.100.106:7125...</div>
    </div>
    <div class="controls">
      <label class="switch" title="Tự Pause nếu phát hiện 2 nhịp 6s liên tiếp G-code đùn > 2.5mm nhưng bánh xe CFS gần như đứng im (<20%)">
        <input type="checkbox" id="autoPauseChk" onchange="toggleAutoPause(this.checked)" />
        Auto-Pause khi hổng nhựa liên tiếp
      </label>
      <button class="btn btn-warn" onclick="sendAction('pause')">PAUSE ngay</button>
      <button class="btn btn-accent" onclick="sendAction('resume')">RESUME</button>
    </div>
  </div>

  <div class="grid-4">
    <div class="card">
      <div class="kpi-label">Lưu lượng tức thời</div>
      <div class="kpi-val" id="kpiFlow">-- mm³/s</div>
      <div class="kpi-sub" id="kpiVel">Tốc độ sợi: -- mm/s</div>
    </div>
    <div class="card">
      <div class="kpi-label">Bánh xe CFS vs G-code (6s)</div>
      <div class="kpi-val" id="kpiRatio">--</div>
      <div class="kpi-sub" id="kpiWheelDelta">dWheel / dE: -- / -- mm</div>
    </div>
    <div class="card">
      <div class="kpi-label">Nhiệt độ Đầu phun / Bàn</div>
      <div class="kpi-val" id="kpiTemp">-- / -- °C</div>
      <div class="kpi-sub" id="kpiSlot">Khay đang chạy: --</div>
    </div>
    <div class="card">
      <div class="kpi-label">Tiến độ &amp; Nhựa còn lại</div>
      <div class="kpi-val" id="kpiLayer">Layer -- / --</div>
      <div class="kpi-sub" id="kpiRem">Còn lại: -- m (-- g)</div>
    </div>
  </div>

  <div class="grid-2">
    <div class="card">
      <h3>
        <span>Biểu đồ lưu lượng tức thời (mm³/s — cập nhật mỗi 1s)</span>
        <span class="sub" id="flowPeak">Max: --</span>
      </h3>
      <canvas id="flowCanvas" width="580" height="210"></canvas>
    </div>
    <div class="card">
      <h3>
        <span>So khớp G-code dE (xanh dương) vs Bánh xe CFS dWheel (xanh lá)</span>
        <span class="sub">Mỗi cột ~6s (EmptyInfo)</span>
      </h3>
      <canvas id="wheelCanvas" width="580" height="210"></canvas>
    </div>
  </div>

  <div class="grid-2">
    <div class="card">
      <h3>
        <span>Dự đoán lượng nhựa cần theo mốc thời gian (G-code + M73)</span>
        <span class="sub" id="estFinish">Còn -- phút</span>
      </h3>
      <table>
        <thead><tr><th>Mốc tương lai</th><th>Khoảng Layer</th><th>Cần thêm</th><th>Tổng tới mốc</th></tr></thead>
        <tbody id="forecastBody"><tr><td colspan="4">Đang tải...</td></tr></tbody>
      </table>
    </div>
    <div class="card">
      <h3>
        <span>Dò hổng nhựa bằng phần mềm &amp; Xuất G-code vá/nối an toàn</span>
        <span class="sub" id="gapCount">0 cảnh báo hổng</span>
      </h3>
      <div class="row-actions" style="margin-bottom:10px;">
        <span class="sub">Tạo G-code nối an toàn (không START_PRINT / G28 Z) từ Layer:</span>
        <input type="number" id="resumeLayerInput" min="1" value="1" />
        <button class="btn btn-accent" onclick="downloadResume()">Tải resume_layer.gcode</button>
      </div>
      <table>
        <thead><tr><th>Lúc</th><th>Layer / Z</th><th>Đùn / Bánh xe</th><th>Thiếu (~mm)</th><th>Hành động</th></tr></thead>
        <tbody id="gapBody"><tr><td colspan="5" style="color:var(--ok);">Chưa phát hiện đoạn hổng nhựa nào từ khi bật monitor.</td></tr></tbody>
      </table>
    </div>
  </div>
</div>

<script>
let layerInitialized = false;

function drawFlowChart(data) {
  const c = document.getElementById('flowCanvas');
  const ctx = c.getContext('2d');
  const w = c.width, h = c.height;
  ctx.clearRect(0, 0, w, h);
  if (!data || data.length < 2) return;
  const maxVal = Math.max(10, ...data.map(d => d.flow)) * 1.1;
  document.getElementById('flowPeak').textContent = 'Đỉnh cửa sổ: ' + Math.max(...data.map(d => d.flow)).toFixed(2) + ' mm³/s';

  ctx.strokeStyle = '#1d2942';
  ctx.lineWidth = 1;
  for (let i = 1; i <= 4; i++) {
    const y = (h / 5) * i;
    ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(w, y); ctx.stroke();
  }

  ctx.beginPath();
  data.forEach((d, idx) => {
    const x = (idx / (data.length - 1)) * (w - 12) + 6;
    const y = h - 14 - (d.flow / maxVal) * (h - 28);
    if (idx === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
  });
  ctx.strokeStyle = '#38bdf8';
  ctx.lineWidth = 2;
  ctx.stroke();

  ctx.lineTo((w - 6), h - 14);
  ctx.lineTo(6, h - 14);
  ctx.closePath();
  ctx.fillStyle = 'rgba(56, 189, 248, 0.14)';
  ctx.fill();

  ctx.fillStyle = '#8ea0bf';
  ctx.font = '11px monospace';
  ctx.fillText(maxVal.toFixed(1) + ' mm³/s', 8, 14);
  ctx.fillText('0', 8, h - 4);
}

function drawWheelChart(data) {
  const c = document.getElementById('wheelCanvas');
  const ctx = c.getContext('2d');
  const w = c.width, h = c.height;
  ctx.clearRect(0, 0, w, h);
  if (!data || data.length === 0) return;
  const slice = data.slice(-24);
  const maxVal = Math.max(5, ...slice.map(d => Math.max(d.d_e, d.d_w))) * 1.15;
  const groupW = (w - 20) / slice.length;
  const barW = Math.max(3, groupW * 0.36);

  slice.forEach((d, idx) => {
    const x0 = 10 + idx * groupW;
    const hE = (d.d_e / maxVal) * (h - 32);
    const hW = (d.d_w / maxVal) * (h - 32);
    ctx.fillStyle = '#38bdf8';
    ctx.fillRect(x0, h - 18 - hE, barW, hE);
    ctx.fillStyle = d.status === 'GAP' ? '#ef4444' : (d.status === 'WARN' ? '#f59e0b' : '#22c55e');
    ctx.fillRect(x0 + barW + 2, h - 18 - hW, barW, hW);
  });
  ctx.fillStyle = '#8ea0bf';
  ctx.font = '11px monospace';
  ctx.fillText(maxVal.toFixed(0) + ' mm', 8, 14);
  ctx.fillText('Lệnh E (xanh dương) vs Bánh xe CFS (xanh lá / đỏ=GAP)', 68, 14);
}

async function refresh() {
  try {
    const res = await fetch('/api/state');
    const s = await res.json();
    if (!s.connected) {
      document.getElementById('statusBadge').textContent = 'DISCONNECTED';
      document.getElementById('fileInfo').textContent = 'Lỗi kết nối: ' + (s.error || '');
      return;
    }
    const badge = document.getElementById('statusBadge');
    badge.textContent = s.overall_state;
    badge.className = 'pill pill-' + s.overall_state;

    document.getElementById('fileInfo').textContent =
      `File: ${s.filename || '-'} | Tiến độ: ${s.progress_pct}% | Z = ${s.z_mm} mm | Cập nhật ${s.updated_at}`;
    document.getElementById('kpiFlow').textContent = `${s.flow.toFixed(2)} mm³/s`;
    document.getElementById('kpiVel').textContent = `Tốc độ sợi: ${s.e_vel.toFixed(2)} mm/s`;

    if (s.latest_wheel) {
      const lw = s.latest_wheel;
      document.getElementById('kpiRatio').textContent = lw.ratio !== null ? `${lw.ratio.toFixed(2)}x (${lw.status})` : lw.status;
      document.getElementById('kpiWheelDelta').textContent = `dWheel / dE: ${lw.d_w} / ${lw.d_e} mm (@ ${lw.t})`;
    }
    document.getElementById('kpiTemp').textContent = `${s.noz_temp}°C / ${s.bed_temp}°C`;
    document.getElementById('kpiSlot').textContent = `Mục tiêu: ${s.noz_target}°/${s.bed_target}°C | Khay: ${s.active_slot}`;
    document.getElementById('kpiLayer').textContent = `Layer ${s.layer} / ${s.layer_count}`;
    if (!layerInitialized && s.layer > 0) {
      document.getElementById('resumeLayerInput').value = s.layer;
      layerInitialized = true;
    }
    document.getElementById('autoPauseChk').checked = !!s.auto_pause_enabled;

    if (s.forecast) {
      document.getElementById('kpiRem').textContent =
        `Còn: ${s.forecast.remaining_m} m (${s.forecast.remaining_g} g) / Tổng ${s.forecast.total_m} m`;
      document.getElementById('estFinish').textContent = `Còn ~${s.forecast.est_remaining_min} phút (M73)`;
      document.getElementById('forecastBody').innerHTML = s.forecast.horizons.map(h =>
        `<tr><td>${h.label}</td><td>~${h.target_layer}</td><td>+${h.add_m} m (+${h.add_g} g)</td><td>${h.cum_m} m</td></tr>`
      ).join('');
    }

    drawFlowChart(s.flow_history);
    drawWheelChart(s.wheel_history);

    document.getElementById('gapCount').textContent = `${s.gap_events.length} cảnh báo hổng`;
    if (s.gap_events.length > 0) {
      document.getElementById('gapBody').innerHTML = s.gap_events.slice().reverse().map(g =>
        `<tr>
          <td>${g.t}</td>
          <td>L${g.layer} / Z${g.z}</td>
          <td>${g.d_e} / ${g.d_w} mm</td>
          <td style="color:var(--bad);font-weight:700;">-${g.missing_mm} mm</td>
          <td><a class="btn btn-bad" href="/api/patch_gcode?mode=gap&start_byte=${g.start_byte}&end_byte=${g.end_byte}&layer=${g.layer}">Tải đoạn vá</a></td>
        </tr>`
      ).join('');
    }
  } catch (e) {
    console.error(e);
  }
}

function downloadResume() {
  const layer = parseInt(document.getElementById('resumeLayerInput').value || '1', 10);
  window.location.href = `/api/patch_gcode?mode=resume&layer=${layer}`;
}

async function toggleAutoPause(enabled) {
  await fetch('/api/config', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({auto_pause: enabled}),
  });
}

async function sendAction(action) {
  if (!confirm(`Xác nhận gửi lệnh ${action.toUpperCase()} tới SPARKX i7?`)) return;
  const r = await fetch('/api/action', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({action}),
  });
  const data = await r.json();
  alert(JSON.stringify(data));
}

refresh();
setInterval(refresh, 1000);
</script>
</body>
</html>
"""


class DashboardHandler(BaseHTTPRequestHandler):
    def log_message(self, format: str, *args) -> None:
        return

    def _send_json(self, obj: dict, code: int = 200) -> None:
        payload = json.dumps(obj, ensure_ascii=False).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(payload)))
        self.end_headers()
        self.wfile.write(payload)

    def do_GET(self) -> None:
        parsed = urllib.parse.urlparse(self.path)
        if parsed.path == "/":
            payload = HTML_PAGE.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(payload)))
            self.end_headers()
            self.wfile.write(payload)
            return
        if parsed.path == "/api/state":
            with MONITOR.lock:
                self._send_json(MONITOR.last_state)
            return
        if parsed.path == "/api/patch_gcode":
            qs = urllib.parse.parse_qs(parsed.query)
            mode = qs.get("mode", ["resume"])[0]
            layer = int(qs.get("layer", ["1"])[0])
            sb = int(qs.get("start_byte", ["0"])[0])
            eb = int(qs.get("end_byte", ["0"])[0])
            if not MONITOR.gcode_index:
                self._send_json({"error": "No active G-code loaded"}, 404)
                return
            gcode_text = MONITOR.gcode_index.build_recovery_gcode(mode, layer, sb, eb)
            fname = f"patch_L{layer}_{mode}.gcode"
            data = gcode_text.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/plain; charset=utf-8")
            self.send_header("Content-Disposition", f'attachment; filename="{fname}"')
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)
            return
        self.send_response(404)
        self.end_headers()

    def do_POST(self) -> None:
        length = int(self.headers.get("Content-Length", "0"))
        body = json.loads(self.rfile.read(length) or b"{}")
        if self.path == "/api/config":
            with MONITOR.lock:
                if "auto_pause" in body:
                    MONITOR.auto_pause_enabled = bool(body["auto_pause"])
                self._send_json({"auto_pause_enabled": MONITOR.auto_pause_enabled})
            return
        if self.path == "/api/action":
            action = body.get("action")
            if action == "pause":
                res = MONITOR._post_gcode("PAUSE")
                self._send_json({"ok": True, "action": "PAUSE", "printer": res})
                return
            if action == "resume":
                res = MONITOR._post_gcode("RESUME")
                self._send_json({"ok": True, "action": "RESUME", "printer": res})
                return
            self._send_json({"error": "Unsupported action"}, 400)
            return
        self.send_response(404)
        self.end_headers()


def poll_loop() -> None:
    while True:
        MONITOR.poll_once()
        time.sleep(1.0)


def main() -> None:
    parser = argparse.ArgumentParser(description="Live Filament Flow Dashboard Server")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    args = parser.parse_args()

    MONITOR.poll_once()
    t = threading.Thread(target=poll_loop, daemon=True)
    t.start()

    srv = ThreadingHTTPServer((args.host, args.port), DashboardHandler)
    print(f"Dashboard listening on http://{args.host}:{args.port}", flush=True)
    srv.serve_forever()


if __name__ == "__main__":
    main()

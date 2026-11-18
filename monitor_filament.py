#!/usr/bin/env python3
"""Live filament feed & volumetric flow monitor for Creality SPARKX i7 (Moonraker)."""

from __future__ import annotations

import argparse
import json
import math
import re
import time
import urllib.request
from datetime import datetime

PRINTER_URL = "http://192.168.100.106:7125"
FILAMENT_AREA_MM2 = math.pi * (1.75 / 2.0) ** 2  # ~2.4053 mm^2
EMPTY_INFO_RE = re.compile(
    r"^\[INFO\]\s+(\d{4}-\d{2}-\d{2}\s+\d{2}:\d{2}:\d{2},\d+)\s+.*?EmptyInfo set_old_position:\s+"
    r"old_pox_x=([-0-9.]+),\s+old_pox_y=([-0-9.]+),\s+old_pox_z=([-0-9.]+),\s+"
    r"old_pox_e=([-0-9.]+),\s+old_measuring_wheel=([-0-9.]+)",
    re.MULTILINE,
)


def fetch_json(path: str) -> dict:
    """Fetch JSON payload from Moonraker."""
    with urllib.request.urlopen(f"{PRINTER_URL}{path}", timeout=3.0) as resp:
        return json.load(resp)


def fetch_latest_wheel(tail_bytes: int = 32768) -> tuple[str, float, float, float] | None:
    """Read the tail of klippy.log using HTTP Range to get latest EmptyInfo."""
    req = urllib.request.Request(
        f"{PRINTER_URL}/server/files/logs/klippy.log",
        headers={"Range": f"bytes=-{tail_bytes}"},
    )
    with urllib.request.urlopen(req, timeout=3.0) as resp:
        text = resp.read().decode("utf-8", errors="replace")
    matches = EMPTY_INFO_RE.findall(text)
    if not matches:
        return None
    ts_str, _x, _y, z_str, e_str, wheel_str = matches[-1]
    return ts_str, float(z_str), float(e_str), float(wheel_str)


def main() -> None:
    parser = argparse.ArgumentParser(description="Live filament feed monitor")
    parser.add_argument("--interval", type=float, default=1.0, help="Poll interval (s)")
    parser.add_argument("--count", type=int, default=0, help="Number of samples (0 = forever)")
    args = parser.parse_args()

    print(
        f"{'Time':<8} | {'Layer':<9} | {'Z(mm)':>6} | {'T_noz':>6} | "
        f"{'E_vel(mm/s)':>11} | {'Flow(mm3/s)':>11} | {'dE_6s':>7} | {'dWheel_6s':>9} | {'Ratio':>6}"
    )
    print("-" * 96)

    prev_wheel: tuple[str, float, float, float] | None = None
    last_d_e = 0.0
    last_d_w = 0.0
    last_ratio = float("nan")
    samples = 0

    while args.count == 0 or samples < args.count:
        status = fetch_json(
            "/printer/objects/query?virtual_sdcard&extruder&heater_bed&motion_report&filament_inventory_manager"
        )["result"]["status"]
        wheel = fetch_latest_wheel()
        if wheel is not None:
            if prev_wheel is not None and wheel[0] != prev_wheel[0]:
                last_d_e = wheel[2] - prev_wheel[2]
                last_d_w = abs(wheel[3] - prev_wheel[3])
                last_ratio = (last_d_w / last_d_e) if last_d_e > 0.2 else float("nan")
            prev_wheel = wheel

        vsd = status["virtual_sdcard"]
        ext = status["extruder"]
        mr = status["motion_report"]
        e_vel = float(mr["live_extruder_velocity"])
        flow = e_vel * FILAMENT_AREA_MM2
        z_pos = float(mr["live_position"][2])
        layer = f"{vsd.get('layer')}/{vsd.get('layer_count')}"
        ratio_str = f"{last_ratio:5.2f}x" if not math.isnan(last_ratio) else "  wait"
        if not math.isnan(last_ratio) and last_d_e > 2.0 and last_ratio < 0.35:
            ratio_str += "!"

        now = datetime.now().strftime("%H:%M:%S")
        print(
            f"{now:<8} | {layer:<9} | {z_pos:6.2f} | {ext['temperature']:6.1f} | "
            f"{e_vel:11.2f} | {flow:11.2f} | {last_d_e:7.2f} | {last_d_w:9.2f} | {ratio_str:>6}",
            flush=True,
        )
        samples += 1
        if args.count == 0 or samples < args.count:
            time.sleep(args.interval)


if __name__ == "__main__":
    main()

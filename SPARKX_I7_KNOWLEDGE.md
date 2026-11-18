# Creality SPARKX i7 — Control, Operations & Troubleshooting Knowledge Base

> Consolidated operational manual, API reference, slicer/firmware caveats, and every real issue encountered while controlling **Creality SPARKX i7** from Mac and **Orange Pi 5 Max** (`orangepi5max`).

---

## 1. Network Endpoints & Host Architecture

- **Printer:** Creality SPARKX i7 (`0.4 mm` nozzle, `260 × 260 mm` Textured PEI build plate, Klipper + Moonraker + Creality CFS-Nano 4-slot filament box)
- **Printer LAN IP:** `192.168.100.106` (MAC `FCEE1901F2E3`)
  - **Moonraker HTTP API:** `http://192.168.100.106:7125`
  - **Moonraker WebSocket:** `ws://192.168.100.106:7125/websocket`
  - **Web UI (Creality Fluidd/Mainsail):** `http://192.168.100.106:4408/`
  - **Open ports:** `80`, `4408`, `7125`, `9999` (`8080`, `8181`, `18188` closed)
  - **Camera / Timelapse limitation:** Moonraker `server/files/roots` only exposes `config`, `logs`, `gcodes`, `config_examples`, `docs`. `webcams` list is empty `[]` and `/webcam/?action=snapshot` returns `502 Bad Gateway` — timelapse frames cannot be pulled via Moonraker HTTP.
- **Orange Pi 5 Max Monitor Host:** `orangepi5max` (Debian Bookworm, RK3588, Python 3.11)
  - **SSH access (from Mac):** `ssh orangepi` (`orangepi@cli-ssh.manhngodh.fun` via `cloudflared` proxy)
  - **Service directory:** `/opt/sparkx-monitor` (`systemd` unit `sparkx-monitor.service`, user `orangepi`)
  - **Dashboard URL:** `http://0.0.0.0:8765` / `http://orangepi5max.tailfe8549.ts.net:8765/`
  - **Alerting:** Discord bot `OpenChamber` -> `#exceptions` channel (`DISCORD_ALERT_CHANNEL_ID=1555209527043952713`), configured in `/etc/default/sparkx-monitor` (`AUTO_RECOVER_ENABLED=0`, `DRY_RUN=1` by default).

---

## 2. How to Control SPARKX i7 via Moonraker API

### A. Read Printer Telemetry & Configs
```bash
# Printer info
curl -s http://192.168.100.106:7125/printer/info

# Live status snapshot (motion, temps, print state, CFS box)
curl -s "http://192.168.100.106:7125/printer/objects/query?print_stats&virtual_sdcard&extruder&heater_bed&motion_report&toolhead&pause_resume&filament_inventory_manager&box"

# List supported Klipper G-code macros/commands
curl -s http://192.168.100.106:7125/printer/gcode/help

# Download firmware configs
curl -s http://192.168.100.106:7125/server/files/config/printer.cfg
curl -s http://192.168.100.106:7125/server/files/config/box.cfg
curl -s http://192.168.100.106:7125/server/files/config/gcode_macro.cfg

# Tail klippy.log efficiently (avoid downloading 40+ MB)
curl -s -H "Range: bytes=-1048576" http://192.168.100.106:7125/server/files/logs/klippy.log
```

**Optimization Note:** Prefer Moonraker WebSocket (`printer.objects.subscribe`) instead of `1s` HTTP `/printer/objects/query` polling. Repeated HTTP polling writes `webhooks:` lines into `klippy.log`, pushing older `measuring_wheel` lines out of fixed-size `Range` tail reads.

### B. Execute G-code, Upload & Start Prints
```bash
# Run G-code command / macro (URL-encoded script)
curl -X POST "http://192.168.100.106:7125/printer/gcode/script?script=M104%20S275"

# Upload G-code file to printer virtual SD
curl -F "file=@my_part.gcode" http://192.168.100.106:7125/server/files/upload

# Start a print
curl -X POST "http://192.168.100.106:7125/printer/print/start?filename=my_part.gcode"

# Pause / Resume (prefer PAUSE over CANCEL_PRINT when recovering!)
curl -X POST "http://192.168.100.106:7125/printer/print/pause"
curl -X POST "http://192.168.100.106:7125/printer/print/resume"
```

### C. Key Machine Coordinates & Thermal Limits
- **Bed area:** `X = 0..260 mm`, `Y = 0..260 mm`
- **Nozzle `max_temp`:** `320°C` (clamp automated purge operations `<= 285°C`, recovery print `<= 280°C`)
- **Waste chute / purge position (`box.cfg`):** `X = 277.0, Y = 130.0` (`pre_cut_pos_x = 5, cut_pos_y = 130`, wipe `X = 258..266, Y = 3..130`). Park before moving to chute should keep `Z >= max(50.0, Z_layer + 15.0)`.
- **Extruder stepper (`printer.cfg`):** `[tmc2209 extruder] run_current: 0.55 A`
- **Z probe (`printer.cfg`):** `[prtouch_v3]` strain-gauge nozzle force sensor (`samples_tolerance: 0.5 mm`). **Do NOT probe onto partially printed plastic** — `START_PRINT` cools nozzle to `140°C` before `G28 Z` / `NOZ_CLEAR` specifically to avoid melting plastic during touch.

---

## 3. Controlling the CFS-Nano Filament Box (`box.cfg`)

The CFS-Nano (`box.T1`, serial485 `/dev/ttyS5` 230400 baud) exposes 4 slots:
- `0` / `T0` = Slot `1A` (`T1A`)
- `1` / `T1` = Slot `1B` (`T1B`)
- `2` / `T2` = Slot `1C` (`T1C`)
- `3` / `T3` = Slot `1D` (`T1D`)

### Built-in Box Macros
- `BOX_INFO_REFRESH ADDR=<addr> NUM=<num>` — triggers `BOX_SET_PRE_LOADING`, reads RFID, reads remaining length.
- `BOX_LOAD_MATERIAL` / `BOX_LOAD_MATERIAL_EXTRUDE_MATERIAL TNN=T1B` — loads slot into extruder and extrudes (`Tn_extrude: 140 mm`).
- `BOX_QUIT_MATERIAL` — heats, cuts filament (`BOX_CUT_MATERIAL`), retracts out of extruder (`Tn_retrude: -20`, `buffer_empty_len: 30`), parks.
- `M8200` toolchange sub-commands:
  - `M8200 P S<next>` (`CR_BOX_PRE_OPT`)
  - `M8200 C` (`CR_BOX_CUT`)
  - `M8200 R` (`CR_BOX_RETRUDE`)
  - `M8200 L I<0..3>` (`CR_BOX_EXTRUDE` mapped to `T1A..T1D`)
  - `M8200 F` (`CR_BOX_FLUSH`)
  - `M8200 O` (`CR_BOX_END_OPT`)

---

## 4. Telemetry Quirks & Real-Time Inflow / Stall Detection

### A. How `measuring_wheel` Works
- The CFS-Nano tracks actual filament pulled via an encoder wheel logged to `klippy.log`:
  - `Tn_data[T1][measuring_wheel]: <float>` — logged every **~1s** while `print_stats.state == "printing"`.
  - `EmptyInfo set_old_position: old_pox_x=..., old_pox_y=..., old_pox_z=..., old_pox_e=..., old_measuring_wheel=...` — logged once per `air_print_detect_e_len` (`20 mm` by default; on slow parts ~0.6 mm/s this only fires every ~47s!).
- **Quantization:** `measuring_wheel` increments in discrete ticks of **`2.3301 mm/tick`**, and its **sign can invert** after manual load/unload! Always measure **`abs(wheel_now - wheel_prev)`**.
- **CRITICAL QUIRK — No `measuring_wheel` updates when `paused`:** When `print_stats.state == "paused"`, Klipper logs `"Air print detect, printer state is paused"` and **stops logging `measuring_wheel`**, nor does `box` JSON expose `measuring_wheel`. During manual/automated purge while `paused`, you can confirm motor `E` stepped, **not** true CFS inflow — inflow can only be verified during the first `8–12s` after resuming `printing`.
- **CRITICAL QUIRK — `virtual_sdcard.layer` is wrong on resumed prints:** Firmware returns cumulative job layer rather than the current `.gcode` file's layer (observed `fw_layer = 230` when file was at `L134`). Always compute `layer` and `layer_count` directly by indexing `;LAYER_CHANGE` / `;Z:` from the active `.gcode` file.

### B. Software Gap Detector Parameters (`/opt/sparkx-monitor`)
- Pair 1s `motion_report.live_position[3]` (from WebSocket) with 1s `Tn_data[T1][measuring_wheel]` (from `klippy.log`) in a 60s ring buffer (`EmptyInfo` alone misses slow-flow stalls).
- **PTFE/buffer lag:** CFS `buffer_empty_len` is `30 mm`, so `1–2s` zero-wheel pauses happen naturally during short moves, retracts, and Z-hops (`dE <= 1–3 mm`). Do **not** trigger at `1–2s`.
- **Reliable thresholds:**
  - **FAST stall:** `6s` window with `dE >= 7.0 mm` and `|dWheel| < 1.0 mm` (`0 ticks`), confirmed on `2` consecutive `2s` checks (`~6–8s` reaction time).
  - **SLOW stall:** `20s` window with `dE >= 6.0 mm` and `|dWheel| < 1.0 mm` (covers slow walls `< 1 mm/s`).
  - **Startup grace:** Ignore first `20s` after print/resume start to absorb initial prime/retract.

---

## 5. Safe Pause & Recovery Rules (Never Break a Print in Progress)

### Hard Safety Rules
1. **NEVER run `CANCEL_PRINT` if you plan to recover the part:** `CANCEL_PRINT` executes `END_PRINT` -> `TURN_OFF_HEATERS` (cools bed -> PETG detaches from Textured PEI) + `M84` (disables steppers -> loses `XYZ` homing). Always use **`PAUSE`**, which leaves `homed_axes: "xyz"` intact and keeps `heater_bed` ON (only cooling nozzle to `140°C` via `PAUSE_SET_TEMPERATURE`).
2. **NEVER include these in a resume/recovery `.gcode`:**
   - `START_PRINT`
   - `G28` / `G28 Z` (probe crashes into the printed part on the bed!)
   - `NOZ_CLEAR`
   - `T0` / `T1` / `T2` / `T3` or `M8200` (would cut/unload filament mid-print or drop temp to `220°C`!)
   - `M5001` (timelapse park per layer wastes ~2.9s/layer and induces ooze/clogs)
3. **Verify `homed_axes` contains `"xyz"`** before any resume move.
4. **Always move `XY` at safe `Z` BEFORE dropping to `Z_layer`:**
   - Raise/stay at `Z >= max(50.0, Z_layer + 15.0)`, travel to `(X_start, Y_start)` at `F12000`, then descend in two steps (`Z_layer + 3.0` -> `Z_layer`).
5. **Preserve `EXCLUDE_OBJECT_DEFINE` and open `EXCLUDE_OBJECT_START`:** Slice any multi-object recovery cleanly so Klipper's object tracking stays consistent.
6. **Note on `EXCLUDE_OBJECT` live during print:** Klipper blocks excluding an object while the toolhead is actively printing inside that object's moves; wait until the toolhead moves onto another object in the layer (or edit/slice a single-object G-code cleanly).
7. **Unverified firmware edge case (needs idle printer test):** Calling `POST /printer/print/start` while state is `paused` may be rejected by Moonraker. `/opt/sparkx-monitor` tries `printer.print.start` -> `SDCARD_PRINT_FILE` -> optional `CLEAR_PAUSE` + start (`ALLOW_CLEAR_PAUSE_FALLBACK=1`) before falling back to `NEEDS_HUMAN`.

---

## 6. All Issues Faced & Root Causes / Fixes

### Issue 1: `flushing` / OrcaSlicer Profile Migration Error (`flush_length_5`, `timelapse_type`, `retraction_distances_when_cut`)
- **Symptom:** Importing Creality Print 7.3 SPARKX i7 `.json` profiles into OrcaSlicer 2.5.0 CLI exited with error `238` / parsing error in `change_filament_gcode`: `"Not a variable name"` on `flush_length_5`.
- **Root Cause:** Creality Print 7.3 defines up to `flush_length_5`, whereas OrcaSlicer only exposes up to `flush_length_4`; Creality Print also used `timelapse_type: -1` and `retraction_distances_when_cut: 28` out of OrcaSlicer bounds.
- **Fix:** Set `timelapse_type = "0"`, `retraction_distances_when_cut = ["18"]`, and clear `change_filament_gcode = ""` for single-material prints (or replace `flush_length_5` references in `change_filament_gcode`).

### Issue 2: OrcaSlicer CLI `--export-3mf` Double-Path Bug & Default Printer Selection
- **Symptom 1:** Passing an absolute path to `--export-3mf` failed (`Unable to open the file .../dir//.../dir/file.3mf.tmp`).
  - **Fix:** Pass only a relative filename to `--export-3mf` alongside `--outputdir <dir>`.
- **Symptom 2:** OrcaSlicer Device tab showed `http://172.23.209.71:4408/` or defaulted to `Bambu Lab A1`.
  - **Fix:** Set `host_type = "octoprint"`, `print_host = "192.168.100.106:7125"`, `print_host_webui = "http://192.168.100.106:4408/"` inside `Creality SPARKX i7 0.4 nozzle (Z-safe).json`, set it as default in `OrcaSlicer.conf`, and patch `.3mf` metadata when needed.

### Issue 3: Bed Temp Override (`80°C` instead of `70°C`) & Redundant Toolchange Cut
- **Symptom:** First sliced G-code emitted `BED_TEMP=80` even after editing filament temp, and ran an unnecessary filament cut (`M8200 C S0`) right after the prime line.
- **Root Cause:** `curr_bed_type` in the machine profile pulled `hot_plate_temp` (`80°C`) while only `textured_plate_temp` had been modified; multi-filament list caused an initial `change_filament_gcode` emission.
- **Fix:** Set `70°C` across **all** plate temp keys (`cool_plate`, `eng_plate`, `textured_plate`, `hot_plate`, `customized_plate`) or override directly in `0.20mm Heavy Load` / `PETG Strong Low-Fan`, set `curr_bed_type = "Textured PEI Plate"`, and pass `"change_filament_gcode": ""` for single-slot prints.

### Issue 4: Prime Line Nozzle Drag / Need for `(Z-safe)` Start G-code
- **Symptom:** Standard purge line dragged/scraped close to the plate on exit.
- **Fix:** Created `Creality SPARKX i7 0.4 nozzle (Z-safe)` machine preset adding safe lift at end of prime line (`G91` / `G1 X-1 Z0.1` / `G1 X-4` / `G92 E0`) and `"time_lapse_gcode": ""` to eliminate `M5001` travel pauses.

### Issue 5: Wrong CFS Slot Mapping & Unrecognized Non-RFID PETG Dropping Temp to `220°C`!
- **Symptom:** Calling `T1` (`1B`) via Moonraker `/printer/print/start` immediately paused (`cfs_lite filament not detected`) or dropped nozzle temp from `272°C` to `220°C` during purge (`get material target temp fail`, `last_type: unknown, last_temp: 220`), followed by `key980: CFS air print detected`.
- **Root Cause:**
  1. Calling Moonraker `/printer/print/start` directly bypasses Creality UI's `print_type` slot-mapping dialog, so hardcoded `T1`/`S1` ran against slot `1B`.
  2. Slot `1B` held non-RFID blue PETG (`unknown` / `remain_len: -1`) while `1C` held RFID `PETG Matte Wood` (`000003`) — `BOX_EXTRUDE_MATERIAL` fell back to `Tn_extrude_temp: 220°C` on `unknown` spools, causing a **cold plug** in the hotend right before Layer 1!
- **Fix:**
  - Either map/assign filament type (`PETG`) to the non-RFID slot on the touchscreen and pre-load (`BOX_LOAD_MATERIAL_EXTRUDE_MATERIAL TNN=T1B`), **or** strip `M8200` / `T0-T3` / `BOX_EXTRUDE_MATERIAL` from start G-code ("Direct PETG" mode) after pre-loading `1B` so the nozzle heats straight to `270–275°C` without a `220°C` drop.
  - Note: Even with `M8200`/`T1` removed from G-code, as long as CFS is connected, firmware `air_print_detect` stays **active** (`20 mm / 6s`) — filament **must** pass through the CFS `measuring_wheel` (bypassing CFS completely triggers `key980` at `20 mm`).

### Issue 6: PETG Nozzle Plugging at `265°C` That Clears at `275°C`
- **Symptom:** Extruder stalled / slipped at `265–270°C` especially on dense solid infill (`Z=14.15..14.30 mm` bridge/solid infill or Layer 14), yet manually pushing at `275°C` cleared it and `265°C` printed normally afterward.
- **Root Causes:**
  1. PLA (`1A`) <-> PETG (`1B`/`1C`) cross-contamination leaving charred PLA residue plugs in the `0.4 mm` nozzle orifice.
  2. Firmware `220–240°C` pre-purge / `nozzle_clean` temperature creating a semi-solid cold plug for high-temp PETG.
  3. Under `0.55 A` extruder current, high-flow dense infill (`6.9–8.0 mm³/s`) spikes backpressure at `265°C` (actual temp dips to `263.9°C`).
- **Fix:** Use **`270–275°C`** for structural PETG prints (`272°C` layer 1 / `270–275°C` other layers, `72–75°C` bed), slow dense speed (`M220 S80` or `wall 80–110 mm/s`), and keep max volumetric flow conservative (`<= 8 mm³/s`).

### Issue 7: Creality Built-in `air_print_retry` Grinds Filament Notches & Late `box.cfg` Detection
- **Symptom 1:** `box.cfg` defaults (`air_print_detect_e_len: 20`, `air_print_detect_time_window: 6`, `air_print_detect_box_len: 0`) allowed `20–28 mm` of stall + `air_print_retry` pushed another `~60 mm` while wheel stayed frozen, **chewing a concave notch** in the filament at the extruder drive gear (`100 mm` pushed at `285°C` still wouldn't extrude until manually fed past the notch!).
- **Symptom 2:** `air_print_detect_box_len: 0` fails to stop partial/intermittent slipping if the wheel ticks even `1` notch (`2.33 mm`) within `6s`.
- **Fix:**
  - Early software `PAUSE` (`6–8s` via Orange Pi monitor) **before** `air_print_retry` grinds the filament.
  - Tuned sensitive `box.cfg` reference saved at `slicing_output/box.cfg.sensitive` (`e_len: 8`, `xyz_len: 5`, `time_window: 14`, `nozzle_block_threshold_percentage: 0.5`).
  - During recovery purge, extrude **at most `18 mm` slowly (`F150`)**. If `dWheel == 0` after 1 retry, **hard-stop (`NEEDS_HUMAN`)** — never keep motor-pushing 40–100 mm!

### Issue 8: Severe PETG Z-Layer Delamination (`Screw` Snapped in 3 Pieces, `Jaw` Legs Broke Cleanly)
- **Symptom:** Parts printed with stock `Generic PETG @SPARKX i7` + `0.20mm Strength` had clean `measuring_wheel` inflow (`91–104%`), yet snapped flat along Z layer lines.
- **Root Cause:** Stock PETG profile blew **`fan_min_speed: 40%`, `fan_max_speed: 90%`, `overhang_fan_speed: 100%` (`M106 S227..S255`)** at `200–230 mm/s` wall speed -> rapid thermal shock froze PETG before interlayer welding.
- **Fix:** Created **`PETG Strong Low-Fan @SPARKX i7 0.4 nozzle`** + **`0.20mm Heavy Load @SPARKX i7 0.4 nozzle`**:
  - Nozzle `272°C / 270–275°C`, Bed `75°C / 72°C`
  - Cooling fan **`10%–25%`**, overhang fan max **`35%`**, fan off for first 3 layers
  - Walls `80 / 110 mm/s`, `wall_loops = 6`, `top/bottom_shell_layers = 6`, `50%–85% Gyroid` infill, `brim_width = 8 mm` (prevents corner warp on long parts like `103 mm` Body).

### Issue 9: Orange Pi Monitor `HTTP Error 416: Requested Range Not Satisfiable` on `klippy.log` Rotation
- **Symptom:** Seen live in `journalctl -u sparkx-monitor`: after printer reboot or Klipper log rotation shrinks `klippy.log` below the cached byte offset in `sparkx_monitor.py`, subsequent HTTP `Range: bytes=<offset>-` polls fail repeatedly with `HTTP Error 416` until the service restarts or resets its offset to `0` (or switches to negative tail `Range: bytes=-1048576` on 416).
- **Fix note for `/opt/sparkx-monitor/sparkx_monitor.py`:** Catch `HTTP 416` (or track `Content-Range` size change), reset byte cursor to `0` / `-1048576`, and re-sync automatically without requiring manual `systemctl restart sparkx-monitor`.

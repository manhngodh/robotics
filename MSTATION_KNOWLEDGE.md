# MStation — Centralized Robotics Knowledge Base (6-Axis Pick-and-Place / Bird's Nest Arm + DC180)

> **Single Source of Truth (SSOT)** for the **MStation** robotics project (`/Users/manhngodh/development/robotics`).
> Every CAD script, hardware assembly, slicer run, web-viewer update, and multi-agent review MUST conform to this document to keep all 6 axes, fasteners, bearings, and clearances synchronized.

For 3D printer operations (Creality SPARKX i7, Klipper/Moonraker, CFS-Nano slots, slicer recovery), see [SPARKX_I7_KNOWLEDGE.md](./SPARKX_I7_KNOWLEDGE.md).

---

## 1. System Overview & 6-Axis Mechanical Chain

- **Application:** 6-DOF desktop robotic arm for bird's nest cleaning / precision picking (*tay robot nhặt yến*), mounted on a desk C-clamp and tipped with a rotary/picking **DC180 end-effector** inside a `u_break` clevis wrist.
- **Actuator Architecture (6 Axes):**
  - **Base & High-Torque Proximal/Shoulder/Elbow Axes (`MG996R`):** Driven by TowerPro / Waveshare `MG996R` servos with **dual-shear U-yokes** (`25T CNC Round Horn` on `-Y` + `623ZZ` ball bearing on `+Y`) and **`≥ 56.0 mm` (`56.50 mm` CAD) inner yoke gap**.
  - **Distal / Wrist Axes (`MG90S` + `u_break` Segments):** Driven by TowerPro `MG90S` micro-servos seated inside `u_break_cover_orig`-compatible pockets bridging down to the `u_break` fork (`02_universal_segment_tab4.step` / `04_HorizontalSegment_Body`).
  - **End-Effector Toolhead (`DC180`):** High-speed 180-size DC spindle motor (`DC180_Drill_*` / `build_segment_dc180_v3.py`) clamped in a two-piece interlocking rail cradle/cap mounted onto the `Tab 4` U-fork.
  - **Rule for 6-Axis Expansion:** Every time a new axis script or link variant is added or modified, update the **Active Canonical Parts** table below and run `.agents/skills/mstation-robotics/scripts/verify_mstation.py` before exporting/slicing.

---

## 2. Canonical Hardware, Dimensions & Fastener Standards

### A. Servos & Motors (Verified CAD Envelopes)

| Component | Key Dimensions (`mm`) | Mounting / Interface | Source CAD |
| :--- | :--- | :--- | :--- |
| **MG996R** | Body `40.7 × 19.7 × 38.7`, tabs `54.0 × 19.7 × 2.6`, spline `+10.35 mm` from center. **Real wire exits at bottom edge on the SAME `+X` end as the output shaft** (`+20.35 mm`, `Z=1.5..5.5` above bottom cap, flush with no `5 mm` boot) | `30.00 mm` pocket depth, pocket `41.8 × 20.4 mm`, `4× Ø2.4 mm` M3 pilot bores (`49.0 × 10.0 mm` pitch); requires `+X` wire-exit relief | [assemble_joint1_demo.py](./assemble_joint1_demo.py), [build_mg996r_to_mg996r.py](./build_mg996r_to_mg996r.py) |
| **MG90S** | Body `22.8 × 12.2 × 22.8`, ears `32.0 × 12.2 × 2.5` (tabs at `Z=16.0..18.5`), `20T` spline at `+4.25 mm` (`Ø4.56`), `-X` 3-wire ribbon | `Ø1.60 mm` M2 pilot bores (`28.0 mm` center pitch in `u_break_cover_orig`, `27.5 mm` nominal U-slots) | [build_mg90s_real.py](./build_mg90s_real.py), [u_break_cover_orig.step](./u_break_cover_orig.step) |
| **DC180 Motor** | Flat-oval body `Ø20.4 × 15.4 × 32.0 mm` (`0.25 mm` print clearance → `r_c=10.45`, `h_c=7.95`), collet `Ø12.5`, collar `Ø6.8` | Front `2× M2` (`Ø2.4 mm` through, `10 mm` pitch `Y=±5.0`) + `4× M3` cradle cap bolts (`Ø3.4` + hex nut traps) | [build_segment_dc180_v3.py](./build_segment_dc180_v3.py) |

### B. Joint Yoke, Bearings, Horns & Fasteners (STRICT INVARIANTS)

| Interface | Mandatory Spec | Reason / Tolerance Rule |
| :--- | :--- | :--- |
| **MG996R 2-Arm Yoke Gap** | **`56.50 mm` nominal (`≥ 56.0 mm` hard minimum)** | `MINUS_Y_IN = -19.50`, `PLUS_Y_IN = +37.00`; symmetric centerline `J2_SHAFT_Y = +8.75 mm`. Accommodates real MG996R + 25T CNC horn + M3 spacer with print tolerance. |
| **Horn Side (`-Y` Arm)** | `4.5 mm` thick (`Y=[-24.0, -19.5]`), `Ø21 mm` lug (`R=10.5`), `Ø6.2` center + `4× Ø2.20 mm` on `Ø14.0 PCD` (`45°` phase) + `Ø4.6×1.5 mm` outer counterbore | Matches **25T Round Plastic Horn (`Ø20 mm`)** & included M2/self-tapping servo box screws (`3.0 mm` web under screw head so short screws reach cleanly into plastic horn). |
| **Bearing Side (`+Y` Arm)** | `5.0 mm` thick (`Y=[+37.0, +42.0]`), `Ø20 mm` lug (`R=10.0`), `1.0 mm` inner lip `Ø7.0 mm` + `4.0 mm` pocket **`Ø10.25 mm`** | Press/slip fit for **`623ZZ` (`3 × 10 × 4 mm`)**. **MUST pair with M3 hardware** (`Ø2.60 mm` M3 pilot bore on mating boss/base & **`OD ≤ 5.5 mm` M3 spacer** so bearing shield is never pinched!). |
| **Base Desk Clamp (`V6`)** | `10–50 mm` desk opening, `Tr16 × 3` leadscrew (`0.35 mm` radial clearance), `4× M4` deck bolts + `3 mm` shear key | Paired solid `V6 Body` + rod-free `V6 Jaw` (`Ø14.70` swivel, `±3.3°` self-stop) + `V6 Screw` + `V6 Joint Pylon`. |

---

## 3. Canonical Coordinate & Orientation Conventions

1. **Dual-Shear Link Frame (`build_mg996r_to_mg996r.py`, `build_mg996r_to_mg90s_2arm.py`):**
   - Joint 1 pivot at **`(PIVOT_X = -6.0, Y = ..., PIVOT_Z)`** (`PIVOT_Z = 24.10` for `Join Link V6`; `PIVOT_Z = 15.50` for `MG996R->MG90S 2-Arm`).
   - Print orientation: flat on **`Z = 0.00 mm`**, `PrintReady_*.stl` centered at `XY = (0, 0)` with `min_z = 0.000`.
2. **Servo Orientation & Wire Routing on Links (CRITICAL):**
   - **Output Spline (`Shaft`) MUST sit at the `+X` outer beveled tip** (`R_corner = sqrt(9^2 + 18^2) = 20.12 mm`, matching `Joint Base V6`).
   - **Real `MG996R` Wire Exit (`+X` Shaft End — Joint Base V6 Style Bay + Lead-in Groove):** Because the 3-wire cable emerges from the bottom edge of the *same* `+X` end as the shaft, provide both a **V6 centered V-arch lead-in groove (`#7b`: `4.2 mm` deep `Y=±3.4→±2.0 mm` from `Z=35.5→22.0` between M3 screws)** so wires tuck in first during drop-in, and a **V6 open-corner wire bay (`X=74.7..85.0, Y=-7.25..+15.25, Z=5.50..22.50`)** matching `Joint Base V6` (`#9`), leaving a `13.0 mm` top M3 deck (`Z=22.5..35.5`) and `9.0 mm` `+Y` beam (`+15.25..+24.25`) solid (`0.000 mm³` collision across `0..±105°`).
3. **Throat Clearance (`BRIDGE_X_MIN`):**
   - With distal nose `R_corner = 20.12 mm`, keep **`BRIDGE_X_MIN ≥ 17.5 mm`** (`≥ 23.5 mm` throat from `PIVOT_X = -6.0`) to guarantee **`0.000 mm³` collision** across **`0° .. ±105°`** with `> 1 mm` FDM print margin.
4. **MG90S Cover (`u_break_cover_orig`) Preservation:**
   - Extract `Solid(max(orig_raw.shells(), key=lambda s: s.area))` so the two native `0.03 mm` internal bubble voids in `u_break_cover_orig.step` are stripped (`OrcaSlicer --info` MUST report `number_of_parts = 1`).
   - Keep bridges strictly inside outer walls (`-2.90..+0.0` / `+15.2..+18.1` relative to head center) so pocket volume diffs inside the servo cavity are **`[]` (`0.000 mm³`)**.
   - Route `MG90S` wires out the **`-Y` side window**, never the `-X` rear window where the proximal servo housing sweeps.

---

## 4. Active Canonical CAD Files (`/Users/manhngodh/development/robotics`)

| Role | Script | Print-Ready STL | Status / Notes |
| :--- | :--- | :--- | :--- |
| **Desk Clamp + J1 Pylon** | [build_arm_base_v6.py](./build_arm_base_v6.py) | `PrintReady_V6_Body/Jaw/Screw/Joint.stl` | ✅ `30.00 mm` pocket depth, `+X` boss **`Ø2.60 mm` (M3)** synced with `623ZZ (3×10×4 mm)`. Note: use `4× M4×16` deck bolts. |
| **MG996R ↔ MG996R Link** | [build_mg996r_to_mg996r.py](./build_mg996r_to_mg996r.py) | [PrintReady_MG996R_to_MG996R.stl](./PrintReady_MG996R_to_MG996R.stl) | ✅ `56.50 mm` yoke gap, `72.00 mm` link, `623ZZ (3×10×4)` + M3 underside bore (`Ø2.60 mm`), shaft `+X`, **V6-style `-Y/+X` wire bay (`Z=5.5..22.5`)**, `BRIDGE_X_MIN=17.5`, `0.000 mm³` sweeps. |
| **MG996R → MG90S (2-Arm)** | [build_mg996r_to_mg90s_2arm.py](./build_mg996r_to_mg90s_2arm.py) | [PrintReady_MG996R_to_MG90S_2Arm.stl](./PrintReady_MG996R_to_MG90S_2Arm.stl) | ✅ `56.50 mm` yoke gap, `57.50 mm` link, `623ZZ (3×10×4)` M3 bearing pocket, `≤ 45°` bottom bevel (`-2.0, 0.0`), `0.000 mm³` sweeps on J1 & J2. |
| **DC180 Toolhead V3** | [build_segment_dc180_v3.py](./build_segment_dc180_v3.py) | [PrintReady_DC180_Housing.stl](./PrintReady_DC180_Housing.stl), [PrintReady_DC180_Cap.stl](./PrintReady_DC180_Cap.stl) | ✅ `Ø20.4×15.4` flat-oval motor cradle + interlocking rail cap + `Tab 4` U-fork. |
| **Hardware Assembly Audits** | [assemble_joint1_demo.py](./assemble_joint1_demo.py), [assemble_2joints_demo.py](./assemble_2joints_demo.py) | [arm_2joints_full_hardware_assembly.stl](./arm_2joints_full_hardware_assembly.stl) | ✅ Full 3× MG996R, `623ZZ`, 25T CNC horn, M3 bearing bolts interference audit. |

---

## 5. Post-Mortem Bug Registry (Anti-Silly-Bug Gates)

Every bug below actually occurred during MStation development. Never ship a CAD revision without checking all six:

1. **Bearing ID vs Mating Bore Mismatch (`624ZZ` 4 mm vs M3 `Ø2.6 mm`):** Switching a link bearing without updating *every* mating pilot bore (`Joint Base V6` + `Join Link` underside) breaks assembly. **Standardized to `623ZZ (3×10×4)` + M3 (`Ø2.60 mm` pilot bore). Gate:** Verify every `623ZZ` pocket (`Ø10.25 × 4.0 mm`) has coaxial `Ø2.60 mm` (`2.5–2.7 mm`) M3 pilot bores on *both* `Joint Base V6` and `Join Link`!
2. **Reversed Distal Servo (`rotate(Axis.Z, 180)`):** Flipping `MG996R #2` backwards put the wire boot at the tip (`+X`) and buried the shaft `39 mm` back, doubling the sweep radius (`40.03` vs `20.12 mm`) and forcing oversized links. **Gate:** Assert `J2_SHAFT_X > J2_BODY_CX` and `NOSE_TIP_X - J2_SHAFT_X <= 19.0 mm`.
3. **Closed Roof Over Wire Boot Blocking Vertical Drop-In:** Leaving a `90°` deck bridge above the `5 mm` `-X` wire boot looked valid at `dz=0`, yet jammed servo insertion by `11–228 mm³`. **Gate:** Run a **`dz = 0..25 mm` vertical insertion intersection test** (`== 0.000 mm³` at every height).
4. **Too-Narrow Yoke Inner Gap (`50.2 mm` vs `≥ 56.0 mm`):** Physical caliper check on real `MG996R + horn + spacer` requires **`≥ 56.0 mm`**. **Gate:** Assert `PLUS_Y_IN - MINUS_Y_IN >= 56.0 mm`.
5. **Bridge Intrusion & Internal STEP Void Shells (`u_break_cover_orig`):** Slicing `u_break_cover_orig.step` after `X=19` cut off its rear wall and let bridges fill the pocket; STEP booleans also left 2 tiny `0.03 mm` bubble shells (`OrcaSlicer number_of_parts = 3`). **Gate:** Probe pocket interior (`== 0.000 mm³` diff) AND run `OrcaSlicer --info` requiring `manifold = yes` and `number_of_parts = 1`.
6. **Zero-Margin `±105°` Graze & Steep Print Overhangs:** `(16.5, 0.0)` grazed `Joint Base` by `0.001 mm³` at `+105°`, and arm bevel `(2.0, 0.0)` created `58°–63°` overhangs. **Gate:** Require `0.000 mm³` across `0°, ±45°, ±90°, ±105°` AND bottom bevel angle `≤ 45°`.

---

## 6. Print & Slicing Baseline (Creality SPARKX i7 — Synced with `SPARKX_I7_KNOWLEDGE.md`)

- **PLA Structural Baseline (confirmed with Manh):** `210°C` nozzle (never exceed `220°C` for PLA), `55°C` Textured PEI bed, **`4` wall loops**, **`15%` infill**, **no support**, **aggressive overhang slowdown**, Slot `1A` (`T1A` / `T0`).
- **Orange Pi Quick Delivery Path:** Copy sliced `.gcode` to `orangepi:/home/orangepi/workspace/robotics/mdstation/`.
- **GitHub / 3D Web Viewer Sync:** Whenever printable STL meshes change and are ready to publish, update `docs/meshes/` + `docs/parts_catalog.json` + `README.md` (only push when asked).

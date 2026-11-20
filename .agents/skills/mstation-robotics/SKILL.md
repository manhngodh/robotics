---
name: mstation-robotics
description: >-
  Centralized CAD, hardware standards, 6-axis kinematics, DC180 toolhead, and
  3D printing workflow for MStation (bird's nest picking robot arm). Activate
  when designing, modifying, assembling, reviewing, or slicing any MStation
  robot part (Joint Base, Join Link, MG996R, MG90S, u_break, DC180, C-clamp).
---

# MStation Robotics Engineering Skill

Use this skill whenever creating, modifying, reviewing, assembling, or slicing parts for **MStation** (`/Users/manhngodh/development/robotics`).

## Step 1: Load the Centralized Knowledge (Mandatory)

Read these canonical sources before making geometry or hardware decisions (do NOT rely on memory):

1. **[MSTATION_KNOWLEDGE.md](/Users/manhngodh/development/robotics/MSTATION_KNOWLEDGE.md)** — Centralized hardware envelopes (`MG996R`, `MG90S`, `DC180`), bearing/horn/bolt standards, dual-shear yoke gaps (`>= 56.0 mm`), coordinate frames, active CAD files, and Post-Mortem Bug Registry.
2. **[SPARKX_I7_KNOWLEDGE.md](/Users/manhngodh/development/robotics/SPARKX_I7_KNOWLEDGE.md)** — Creality SPARKX i7 Klipper/Moonraker API, CFS-Nano slots, nozzle limits, and slicing baseline (`PLA 210°C`, `4` wall loops, `15%` infill, no supports, slow overhangs).

## Step 2: Enforce Non-Negotiable Cross-Part Invariants

- **MG996R Yoke Gap:** Never make `PLUS_Y_IN - MINUS_Y_IN < 56.0 mm` (canonical `56.50 mm`, `MINUS_Y_IN = -19.50`, `PLUS_Y_IN = +37.00`, `J2_SHAFT_Y = +8.75`).
- **Bearing <-> Fastener Match:** `623ZZ` (`3 × 10 × 4 mm`, `Ø10.25 × 4.0 mm` pocket, `1.0 mm` `Ø7.0 mm` inner lip) MUST ALWAYS pair with **`M3` (`Ø2.60 mm` pilot bore)** and **`OD <= 5.5 mm` spacer**. Never mix `M3` (`Ø2.6 mm`) and `M4` (`Ø3.4 mm` / `624ZZ`) across mating joints.
- **Horn Arm Hardware:** `-Y` `4.5 mm` arm uses `4× Ø2.20 mm` bores on `Ø14.0 mm PCD` + `Ø4.6×1.5 mm` outer counterbore (`3.0 mm` clamped web) for the included `25T` round plastic horn & servo box screws.
- **Servo Orientation on Links:** Output spline MUST be at the `+X` beveled tip (`R_corner <= 20.12 mm`); rubber wire boot MUST face `-X` through a full-height slot so the servo drops in vertically without jamming.
- **MG90S Cover & Wire Routing:** Extract only outer shell of `u_break_cover_orig.step` (`Solid(max(shells, key=area))`), preserve the native pocket unobstructed, and route wires out the `-Y` side window (never `-X`).
- **Printability & Swing Range:** Bottom bevels `<= 45°`, `0.000 mm³` clash across `0°..±105°`, flat `min_z = 0.000`, `manifold = yes`, `number_of_parts = 1`.

## Step 3: Run the Automated Gate After Every Edit

```bash
python3 /Users/manhngodh/development/robotics/.agents/skills/mstation-robotics/scripts/verify_mstation.py
```

And execute any edited CAD / assembly script directly (`build_*.py`, `assemble_joint1_demo.py`, `assemble_2joints_demo.py`) plus visually inspect rendered PNGs before reporting completion.

## Step 4: Keep Knowledge Synchronized

Whenever a new axis, motor, bearing, bolt size, or link dimension is introduced or changed, update **[MSTATION_KNOWLEDGE.md](/Users/manhngodh/development/robotics/MSTATION_KNOWLEDGE.md)** in the same edit so no future turn or subagent falls out of sync.

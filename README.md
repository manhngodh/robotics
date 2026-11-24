# 2-Joint MG996R Robot Arm & Desk Clamp CAD

Parametric Python (`build123d`) CAD models, printable `.stl` / `.step` parts, hardware interference verification scripts, and an interactive Three.js 3D Assembly, Part Inspector & 3D Measurement Viewer.

## Interactive 3D Web Viewer (GitHub Pages)
- **Live Viewer:** https://manhngodh.github.io/robotics/
  - **Xem từng Part (13):** Inspect individual parts (`Join Link V6`, `2-Arm MG996R→MG90S`, `Joint Base V6`, `C-Clamp`, `DC180`, `MG996R`, `MG90S`, `623ZZ`, `25T Horn`) centered with bounding dimensions & specs.
  - **Lắp ráp 3 Khớp:** Articulate Joint 1, Joint 2 & Joint 3 (Wrist MG90S) across `±105°`, Explode view (`0–100%`), and X-Ray mode.
  - **3D Measurement Tool (`M`):** Measure 2-point distance (`d`, `ΔX/ΔY/ΔZ`), 3-point hole diameter (`Ø`), and bounding box (`XYZ`) directly in the browser.

## Key Printable Parts (Click on GitHub for Native 3D STL Preview)
- [PrintReady_MG996R_to_MG996R.stl](PrintReady_MG996R_to_MG996R.stl) — Monolithic MG996R-to-MG996R link (`100.50 × 66.00 × 36.50 mm`, `72.00 mm` J1→J2, `56.50 mm` yoke gap, `623ZZ` `3×10×4` M3 + `Ø5.5×1.0 mm` underside spigot, shaft `+X`, V6 `-Y/+X` wire bay + `#7b` lead-in groove, `0.000 mm³` clash across `±105°`)
- [PrintReady_MG996R_to_MG90S_2Arm.stl](PrintReady_MG996R_to_MG90S_2Arm.stl) — Compact 2-Arm MG996R-to-MG90S cover (`78.50 × 66.00 × 26.00 mm`, `57.50 mm` link length, `56.50 mm` yoke gap, `623ZZ` M3, `≤45°` bottom bevel, `0.000 mm³` clash across `±105°`)
- [PrintReady_V6_Joint.stl](PrintReady_V6_Joint.stl) — V6 Base-to-MG996R Joint Pylon (`30.00 mm` pocket depth, `+X` M3 `Ø2.60 mm` spigot)
- [PrintReady_V6_Body.stl](PrintReady_V6_Body.stl) — V6 Desk Clamp Body (`M8` captive hex nut trap `13.3 mm AF` + `Ø8.6 mm` bore)
- [PrintReady_V6_Jaw.stl](PrintReady_V6_Jaw.stl) — V6 Moving Clamp Jaw (rod-free solid block, `Ø8.8 mm` M8 tip seat)
- [PrintReady_V6_M8_Knob.stl](PrintReady_V6_M8_Knob.stl) — V6 Threaded M8×1.25 Knob (`Ø41.4 × 18 mm`, `3 mm` closed bottom + 2× radial M3 lock bores for headless `M8 × 120` rod)
- [PrintReady_V6_M8_Nut.stl](PrintReady_V6_M8_Nut.stl) — V6 Printable Backup `M8×1.25` Flanged Hex Nut (`13.0 mm AF`)
- [PrintPlate_V6_BaseAndJoint.stl](PrintPlate_V6_BaseAndJoint.stl) — Combined V6 Base + Joint Print Plate
- [arm_2joints_full_hardware_assembly.stl](arm_2joints_full_hardware_assembly.stl) — Full 2-Joint Assembly with 3× MG996R, 3× 623ZZ (`3×10×4 mm`), 3× 25T CNC Horns & M3 bolts

## Centralized Knowledge & Main CAD Scripts
- [MSTATION_KNOWLEDGE.md](MSTATION_KNOWLEDGE.md) — Centralized MStation 6-axis + DC180 mechanical truth & invariants
- [build_mg996r_to_mg996r.py](build_mg996r_to_mg996r.py) — Join Link V6 generator & `±105°` sweep check
- [build_mg996r_to_mg90s_2arm.py](build_mg996r_to_mg90s_2arm.py) — 2-Arm MG996R-to-MG90S cover generator & J1/J2 sweeps
- [build_arm_base_v6.py](build_arm_base_v6.py) — V6 Desk C-Clamp + Joint Base Pylon generator
- [assemble_joint1_demo.py](assemble_joint1_demo.py) — Joint 1 hardware interference audit
- [assemble_2joints_demo.py](assemble_2joints_demo.py) — Full 2-Joint swing sweep & assembly export

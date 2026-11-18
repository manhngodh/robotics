# 2-Joint MG996R Robot Arm & Desk Clamp CAD

Parametric Python (`build123d`) CAD models, printable `.stl` / `.step` parts, hardware interference verification scripts, and an interactive Three.js 3D Assembly, Part Inspector & 3D Measurement Viewer.

## Interactive 3D Web Viewer (GitHub Pages)
- **Live Viewer:** https://manhngodh.github.io/robotics/
  - **Xem từng Part (9):** Inspect individual parts centered with bounding dimensions & specs.
  - **Lắp ráp 2 Khớp:** Articulate Joint 1 & Joint 2 across `±105°`, Explode view (`0–100%`), and X-Ray mode.
  - **3D Measurement Tool (`M`):** Measure 2-point distance (`d`, `ΔX/ΔY/ΔZ`), 3-point hole diameter (`Ø`), and bounding box (`XYZ`) directly in the browser.

## Key Printable Parts (Click on GitHub for Native 3D STL Preview)
- [PrintReady_MG996R_to_MG996R.stl](PrintReady_MG996R_to_MG996R.stl) — Monolithic MG996R-to-MG996R link (`116.50 × 60.70 × 35.60 mm`, `66.00 mm` J1→J2, `30.00 mm` pocket depth, `PIVOT_Z = 24.10 mm` coaxial alignment, lower `+X` 45° wire exit arch, `0.000 mm³` clash across `±105°`)
- [PrintReady_V6_Joint.stl](PrintReady_V6_Joint.stl) — V6 Base-to-MG996R Joint Pylon (`30.00 mm` pocket depth)
- [PrintReady_V6_Body.stl](PrintReady_V6_Body.stl) — V6 Desk Clamp Body
- [PrintReady_V6_Jaw.stl](PrintReady_V6_Jaw.stl) — V6 Moving Clamp Jaw
- [PrintReady_V6_Screw.stl](PrintReady_V6_Screw.stl) — V6 Trapezoidal Clamp Screw
- [PrintPlate_V6_BaseAndJoint.stl](PrintPlate_V6_BaseAndJoint.stl) — Combined V6 Base + Joint Print Plate
- [arm_2joints_full_hardware_assembly.stl](arm_2joints_full_hardware_assembly.stl) — Full 2-Joint Assembly with 3× MG996R, 3× 624ZZ (`4×13×5 mm`), 3× 25T CNC Horns & M4 bolts

## Main Parametric CAD Scripts
- [build_mg996r_to_mg996r.py](build_mg996r_to_mg996r.py) — Join Link generator & Joint 1 ±105° sweep check
- [build_arm_base_v6.py](build_arm_base_v6.py) — V6 Desk C-Clamp + Joint Base Pylon generator
- [assemble_joint1_demo.py](assemble_joint1_demo.py) — Joint 1 hardware interference audit
- [assemble_2joints_demo.py](assemble_2joints_demo.py) — Full 2-Joint swing sweep & assembly export

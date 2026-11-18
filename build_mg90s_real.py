#!/usr/bin/env python3
"""
Build an accurate, detailed, real-world TowerPro MG90S micro-servo CAD model.
Local coordinate conventions (compatible with M Robot assembly):
  - Housing main body: X=[-11.4, +11.4] (22.8 mm), Y=[-6.1, +6.1] (12.2 mm), Z=[0.0, 22.8]
  - Mounting tabs (ears): X=[-16.0, +16.0] (32.0 mm), Y=[-6.1, +6.1], Z=[16.0, 18.5]
      * Open U-slots Ø2.2 mm centered at X=±13.75 mm (27.5 mm pitch)
      * Underside of tabs at Z=16.0 mm rests flush on u_break_cover (Z=22.0) / pylon (X=-21.5)
  - Gear turret: main Ø11.8 mm at (4.25, 0) Z=[22.8, 26.0] + offset Ø5.8 mm idler boss at (-1.65, 0) Z=[22.8, 25.5]
      * Bearing step Ø6.8 mm at (4.25, 0) Z=[26.0, 26.6]
  - Output 20-tooth spline shaft: center (4.25, 0), Z=[26.6, 29.0], Ø4.56 mm tip / Ø4.04 mm root, M2.5 center hole
  - 3-wire ribbon tail stub exiting -X strain-relief grommet (brown / red / orange)
"""
import math
import os
from build123d import *

OUT_DIR = "/Users/manhngodh/development/robotics"

SHAFT_X = 4.25
BODY_HALF_X = 11.4
BODY_HALF_Y = 6.1
TAB_HALF_X = 16.0
HOLE_PITCH_HALF = 13.75
TAB_Z_BOT = 16.0
TAB_Z_TOP = 18.5
CASE_Z_TOP = 22.8


def _rounded_rect_sketch(hx, hy, r_corner):
    with BuildSketch(Plane.XY) as sk:
        RectangleRounded(hx * 2.0, hy * 2.0, r_corner)
    return sk.sketch


def build_mg90s_case():
    with BuildPart() as p:
        # 1. Main rectangular servo body (Z = 0 .. 22.8 mm) with rounded corners
        with BuildSketch(Plane.XY):
            RectangleRounded(BODY_HALF_X * 2.0, BODY_HALF_Y * 2.0, 0.9)
        extrude(amount=CASE_Z_TOP)

        # Chamfer bottom and top perimeter edges cleanly
        try:
            chamfer(p.edges().filter_by(lambda e: abs(e.center().Z) < 0.05), length=0.35)
        except Exception:
            pass
        try:
            chamfer(p.edges().filter_by(lambda e: abs(e.center().Z - CASE_Z_TOP) < 0.05), length=0.35)
        except Exception:
            pass

        # 2. Horizontal shell seam grooves (bottom cap @ Z=4.3, top cap @ Z=18.6)
        for z_seam in (4.3, 18.6):
            with Locations((0, 0, z_seam)):
                Box(BODY_HALF_X * 2.0 + 1.0, BODY_HALF_Y * 2.0 + 1.0, 0.32, mode=Mode.SUBTRACT)
            # Re-fill slightly inset core so groove is 0.25 mm deep all around
            with BuildSketch(Plane.XY.offset(z_seam - 0.16)):
                RectangleRounded((BODY_HALF_X - 0.25) * 2.0, (BODY_HALF_Y - 0.25) * 2.0, 0.7)
            extrude(amount=0.32)

        # 3. Mounting tabs (flanges) at Z = 16.0 .. 18.5 mm, span 32.0 x 12.2 mm
        with BuildSketch(Plane.XY.offset(TAB_Z_BOT)):
            RectangleRounded(TAB_HALF_X * 2.0, BODY_HALF_Y * 2.0, 0.9)
        extrude(amount=(TAB_Z_TOP - TAB_Z_BOT))

        # U-slot screw holes centered at X = ±13.75 mm (Ø2.2 mm open outward to ±16.0 mm)
        for sx in (-1, 1):
            cx = sx * HOLE_PITCH_HALF
            with Locations((cx, 0, 0.5 * (TAB_Z_BOT + TAB_Z_TOP))):
                Cylinder(radius=1.1, height=(TAB_Z_TOP - TAB_Z_BOT) + 1.0, mode=Mode.SUBTRACT)
            with Locations((cx + sx * 1.5, 0, 0.5 * (TAB_Z_BOT + TAB_Z_TOP))):
                Box(3.0, 2.2, (TAB_Z_TOP - TAB_Z_BOT) + 1.0, mode=Mode.SUBTRACT)

        # 4. Side label pockets on +Y and -Y faces (authentic MG90S recessed label panel)
        for sy in (-1, 1):
            with Locations((0, sy * (BODY_HALF_Y - 0.08), 10.25)):
                Box(16.6, 0.24, 8.2, mode=Mode.SUBTRACT)

        # 5. Four bottom case-screw dimples on Z=0
        for sx in (-1, 1):
            for sy in (-1, 1):
                with Locations((sx * 9.5, sy * 4.3, 0.2)):
                    Cylinder(radius=0.85, height=0.6, mode=Mode.SUBTRACT)

        # 6. Wire strain-relief grommet at -X end (Z = 1.6 .. 4.3 mm)
        with Locations((-BODY_HALF_X - 0.5, 0, 2.95)):
            Box(1.2, 4.8, 2.5, mode=Mode.ADD)

        # 7. Iconic Twin-Cylinder Gearbox Turret on top of Z = 22.8 mm
        # Main output gear turret: Ø11.8 mm at (SHAFT_X, 0), Z = 22.6 .. 26.0 mm
        with Locations((SHAFT_X, 0, 22.6 + 3.4 / 2.0)):
            Cylinder(radius=5.9, height=3.4)
        # Secondary idler gear boss: Ø5.8 mm at (-1.65, 0), Z = 22.6 .. 25.4 mm
        with Locations((-1.65, 0, 22.6 + 2.8 / 2.0)):
            Cylinder(radius=2.9, height=2.8)
        # Bearing shoulder ring: Ø6.8 mm at (SHAFT_X, 0), Z = 25.9 .. 26.6 mm
        with Locations((SHAFT_X, 0, 26.25)):
            Cylinder(radius=3.4, height=0.7)

        # Subtle top chamfer on turret & idler edges
        for z_top, r_c in [(26.0, 5.9), (25.4, 2.9), (26.6, 3.4)]:
            try:
                chamfer(
                    p.edges().filter_by(
                        lambda e, zt=z_top: abs(e.center().Z - zt) < 0.05 and e.length > 5.0
                    ),
                    length=0.22,
                )
            except Exception:
                pass

    part = p.part
    part.label = "MG90S_Case"
    part.color = Color(0.12, 0.44, 0.75)  # TowerPro translucent-style cobalt blue
    return part


def build_mg90s_spline(n_teeth=20, r_tip=2.28, r_root=2.02, z_bot=26.5, z_top=29.0):
    pts = []
    for i in range(n_teeth * 2):
        ang = math.pi * i / n_teeth
        r = r_tip if (i % 2 == 0) else r_root
        pts.append((SHAFT_X + r * math.cos(ang), r * math.sin(ang)))
    with BuildPart() as p2:
        with BuildSketch(Plane.XY.offset(z_bot)):
            with BuildLine():
                Polyline(*pts, close=True)
            make_face()
        extrude(amount=(z_top - z_bot))
        # M2.5 center screw bore + countersink
        with Locations((SHAFT_X, 0, z_top - 1.8)):
            Cylinder(radius=1.05, height=3.8, mode=Mode.SUBTRACT)
        with Locations((SHAFT_X, 0, z_top - 0.18)):
            Cone(bottom_radius=1.05, top_radius=1.35, height=0.38, mode=Mode.SUBTRACT)

    part = p2.part
    part.label = "MG90S_Spline"
    part.color = Color(0.76, 0.79, 0.82)  # Machined metal steel/aluminum
    return part


def build_mg90s_wires():
    # Three parallel curved wire stubs (brown, red, orange) exiting grommet (-12.0, y, 2.95)
    specs = [
        ("MG90S_Wire_Brown",  -1.15, Color(0.38, 0.22, 0.12)),
        ("MG90S_Wire_Red",     0.00, Color(0.78, 0.16, 0.14)),
        ("MG90S_Wire_Orange", +1.15, Color(0.92, 0.50, 0.10)),
    ]
    wires = []
    for name, y_off, col in specs:
        with BuildPart() as wp:
            with BuildLine() as ln:
                Spline(
                    (-11.8, y_off, 2.95),
                    (-14.2, y_off, 2.60),
                    (-15.6, y_off, 1.00),
                )
            with BuildSketch(Plane.YZ.offset(-11.8)) as sk:
                with Locations((y_off, 2.95)):
                    Circle(radius=0.52)
            sweep(path=ln.line)
        part = wp.part
        part.label = name
        part.color = col
        wires.append(part)
    return wires


def main():
    case = build_mg90s_case()
    spline = build_mg90s_spline()
    wires = build_mg90s_wires()

    all_parts = [case, spline] + wires
    for prt in all_parts:
        bb = prt.bounding_box()
        print(f"  {prt.label:18s}: X=[{bb.min.X:.2f},{bb.max.X:.2f}], Y=[{bb.min.Y:.2f},{bb.max.Y:.2f}], Z=[{bb.min.Z:.2f},{bb.max.Z:.2f}]")

    export_stl(case, os.path.join(OUT_DIR, "mg90s_real_case.stl"))
    export_stl(spline, os.path.join(OUT_DIR, "mg90s_real_spline.stl"))
    export_stl(Compound(children=wires), os.path.join(OUT_DIR, "mg90s_real_wires.stl"))

    asm = Compound(children=all_parts)
    step_path = os.path.join(OUT_DIR, "mg90s_real.step")
    export_step(asm, step_path)
    print(f"Exported {step_path}")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
Build widened Cover<2> arm link (u_break_cover_mg996r) matched symmetrically
to the dual-shear MG996R Join Base (arm_base_v6_joint):
- Keeps upper Cover<3>/MG90S #2 housing (X_loc >= +21) 100% original.
- Keeps -Y arm at Y_loc = [-22.0, -17.5] (X_w = -37.0..-32.5), fills old MG90S
  horn pocket, adds Ø21 mm mounting pad with Ø6.2 center bore + 4x Ø3.2 mm M3
  holes on Ø14.0 mm PCD (45 deg phase) for standard 25T aluminum CNC round horn.
- Shifts +Y bearing arm +13.2 mm to Y_loc = [+30.7, +35.2] (X_w = +15.7..+20.2),
  preserving the native Ø8.2 mm (683ZZ bearing) + Ø6.6 mm inner lip interface
  coaxial with arm_base_v6_joint's +X bearing spigot (+13.5..+15.5).
- Bridges & gussets the +13.2 mm extension cleanly into the upper body.
"""

import os
import math
from build123d import *

DIR = "/Users/manhngodh/development/robotics"
ORIG_STEP = os.path.join(DIR, "u_break_cover_orig.step")

PIVOT_X = -6.0
PIVOT_Z = 15.5
SHIFT_PLUS_Y = 13.2  # Widens inner gap from 35.0 mm to 48.2 mm


def build_widened_cover():
    orig = import_step(ORIG_STEP)

    # 1. Original upper body + -Y arm (slice away only original +Y arm at Y > +17.4)
    left_and_body = Compound(orig.intersect(Pos(-14.5 + 40.0, -24.0 + 20.7, 12.0) * Box(85.0, 41.4, 30.0)))

    # 2. Original +Y bearing arm (Y >= +17.45, X <= +20.5), shifted +13.2 mm in +Y
    plus_y_arm = Compound(orig.intersect(Pos(2.5, 17.45 + 5.0, 12.0) * Box(36.0, 10.0, 30.0)))
    plus_y_arm_shifted = plus_y_arm.translate((0, SHIFT_PLUS_Y, 0))

    # 3. Structural bridge + tapered shoulder gusset linking original Y=17.4 out to Y=30.7
    with BuildPart() as bridge:
        # Straight crossbar span (X=15.0..19.5, Y=17.0..31.0, Z=0.0..21.0)
        with Locations((17.25, 24.0, 10.5)):
            Box(4.5, 14.0, 21.0)
        # Tapered gusset on upper side (X=19.5..28.5 -> Y=31.0..10.3, Z=0.0..21.0)
        with BuildSketch(Plane.XY) as sk_g:
            with BuildLine():
                Polyline(
                    (19.4, 10.3),
                    (19.4, 31.0),
                    (28.5, 10.3),
                    (19.4, 10.3),
                )
            make_face()
        extrude(sk_g.sketch, amount=21.0)

    # 4. Upgrade -Y arm (Y=-22.0..-17.5) for 25T Ø20 mm CNC round servo horn
    with BuildPart() as horn_pad:
        # Solid fill of old MG90S horn pocket + Ø21 mm mounting pad across Y=[-22.0, -17.5]
        with Locations((PIVOT_X, -19.75, PIVOT_Z)):
            Cylinder(radius=10.5, height=4.5, rotation=(90, 0, 0))

    # Fuse parts together
    part = (left_and_body + plus_y_arm_shifted + bridge.part + horn_pad.part)

    # Cut clean 25T round horn bolt pattern on -Y arm (Y=-22.5..-17.0)
    with BuildPart() as horn_cuts:
        # Center Ø6.2 mm bore for MG996R spline boss / M3 screw
        with Locations((PIVOT_X, -19.75, PIVOT_Z)):
            Cylinder(radius=3.1, height=6.0, rotation=(90, 0, 0))
        # 4x Ø3.2 mm M3 bolt holes on Ø14.0 mm PCD (R=7.0 mm at 45, 135, 225, 315 deg)
        for k in range(4):
            ang = math.radians(45.0 + k * 90.0)
            hx = PIVOT_X + 7.0 * math.cos(ang)
            hz = PIVOT_Z + 7.0 * math.sin(ang)
            with Locations((hx, -19.75, hz)):
                Cylinder(radius=1.6, height=6.0, rotation=(90, 0, 0))

    part = part - horn_cuts.part

    # Clean redundant inner faces
    try:
        part = part.clean()
    except Exception:
        pass
    return part


def place_in_world(cover, shaft_z=96.0, angle_deg=0.0):
    """
    Place local u_break_cover_mg996r onto world arm_base_v6_joint frame:
      X_w = Y_loc - 15.0
      Y_w = Z_loc - 15.5
      Z_w = (X_loc + 6.0) + shaft_z
    Then rotate around world pivot axis (-15.0, 0.0, shaft_z) along X by angle_deg.
    """
    # Map (X_loc, Y_loc, Z_loc) -> (Y_loc, Z_loc, X_loc): Ry(-90) then Rz(-90)? Let's apply exact matrix via Plane or rotations:
    # Step 1: Rotate +90 around X: (x, y, z) -> (x, -z, y)
    # Wait: test via rotate(Axis.Z, 90) * rotate(Axis.Y, -90):
    # Ry(-90): (x,y,z) -> (-z, y, x); then Rz(90): (-z, y, x) -> (-y, -z, x). Not right.
    # Try Rz(-90) * Ry(-90): Ry(-90): (x,y,z)->(-z, y, x); Rz(-90): (-z, y, x)->(y, z, x)! EXACT!
    p = cover.translate((-PIVOT_X, 0.0, -PIVOT_Z))
    p = p.rotate(Axis.Y, -90).rotate(Axis.Z, -90)
    p = p.translate((-15.0, 0.0, shaft_z))
    if abs(angle_deg) > 1e-6:
        p = p.rotate(Axis((-15.0, 0.0, shaft_z), (1, 0, 0)), angle_deg)
    return p


def main():
    print("Building widened u_break_cover_mg996r...")
    cover = build_widened_cover()
    bb = cover.bounding_box()
    print(f"  u_break_cover_mg996r local bbox: X=[{bb.min.X:.2f},{bb.max.X:.2f}], "
          f"Y=[{bb.min.Y:.2f},{bb.max.Y:.2f}], Z=[{bb.min.Z:.2f},{bb.max.Z:.2f}]")
    print(f"  is_valid={cover.is_valid}, solids={len(cover.solids())}, vol={cover.volume:.1f} mm3")

    step_path = os.path.join(DIR, "u_break_cover_mg996r.step")
    stl_path = os.path.join(DIR, "u_break_cover_mg996r.stl")
    export_step(cover, step_path)
    export_stl(cover, stl_path)

    # Print-ready orientation: flat back (X_loc=60.5? No, flat bottom Z_loc=0 on bed!)
    pr_bb = cover.bounding_box()
    print_ready = cover.translate((-0.5 * (pr_bb.min.X + pr_bb.max.X), -0.5 * (pr_bb.min.Y + pr_bb.max.Y), -pr_bb.min.Z))
    export_stl(print_ready, os.path.join(DIR, "PrintReady_Cover2_MG996R.stl"))

    # Full 3D collision & coaxial check against arm_base_v6_joint
    from build_arm_base_v6 import SHAFT_Z, BODY_CENTER_Z
    joint = import_step(os.path.join(DIR, "arm_base_v6_joint.step"))
    # Build simplified virtual MG996R (body 40.7x19.7x38.0 + tabs 54x20x2.6 + spline up to -34.5)
    with BuildPart() as mg_sim:
        with Locations((-18.5 + 9.0, 0, BODY_CENTER_Z)):  # back half -18.5..+9.0 (27.5 behind tabs)
            Box(27.5, 19.7, 40.7)
        with Locations((-18.5 - 5.6, 0, BODY_CENTER_Z)):  # front half -29.7..-18.5 (11.2 to gear face)
            Box(11.2, 19.7, 40.7)
        with Locations((-18.5 - 1.3, 0, BODY_CENTER_Z)):  # 2.6 mm tabs seated flush on HEAD_X_FRONT=-18.5 (-21.1..-18.5)
            Box(2.6, 19.7, 54.0)
        with Locations((-18.5 - 13.6, 0, SHAFT_Z)):  # spline boss + shaft (-29.7..-34.5)
            Cylinder(radius=2.95, height=4.8, rotation=(0, 90, 0))

    print(f"\nCollision check against arm_base_v6_joint (SHAFT_Z={SHAFT_Z}) across swing angles:")
    for ang in [0.0, 45.0, 90.0, 105.0, -45.0, -90.0, -105.0]:
        cw = place_in_world(cover, shaft_z=SHAFT_Z, angle_deg=ang)
        inter = cw.intersect(joint)
        vol = sum(s.volume for s in inter.solids()) if (inter is not None and len(inter) > 0) else 0.0
        inter_mg = cw.intersect(mg_sim.part)
        vol_mg = sum(s.volume for s in inter_mg.solids()) if (inter_mg is not None and len(inter_mg) > 0) else 0.0
        print(f"  angle={ang:+6.1f} deg -> joint overlap={vol:.3f} mm3 | mg996r overlap={vol_mg:.3f} mm3")

    # Export visual assembly check STEP (0 deg upright)
    cw0 = place_in_world(cover, shaft_z=SHAFT_Z, angle_deg=0.0)
    asm = Compound(children=[joint, cw0, mg_sim.part])
    export_step(asm, os.path.join(DIR, "arm_base_v6_mg996r_mating_check.step"))
    print("Saved u_break_cover_mg996r.step/.stl & arm_base_v6_mg996r_mating_check.step")


if __name__ == "__main__":
    main()

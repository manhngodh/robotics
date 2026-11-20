#!/usr/bin/env python3
"""
Build monolithic parametric Cover<2> arm link upgrading BOTH ends to MG996R:
  Joint 1 (bottom, local Y axis at (-6.0, 0.0, 15.5)):
    - Dual-shear U-yoke matching arm_base_v6_joint 100%:
      * -Y arm (Y=[-22.0, -17.5]): Ø21 mm pad, Ø6.2 center + 4x Ø3.2 M3 holes on Ø14.0 PCD (25T CNC horn)
      * +Y arm (Y=[+30.7, +35.2]): Ø8.2 mm 683ZZ bearing pocket + Ø6.6 mm inner lip (0.2 mm clear of +X spigot)
      * 8.0 mm solid crossbar bridge (X=[+15.5, +23.5], Z=[0, 22]) clearing Joint 1 head across +/-105 deg

  Joint 2 (top, orthogonal 90 deg, local +Z axis at (72.0, 0.0) -> 78.0 mm link center-to-center):
    - Top-open monolithic MG996R #2 housing (Z=[0, 34.0], prints flat on Z=0 with ZERO internal overhang):
      * Pocket: 41.3 x 20.4 x 28.0 mm (X=[41.00, 82.30], Y=[-10.20, +10.20], Z=[6.0, 34.0])
      * 5.3 mm solid side walls (Y=+/-15.5 -> 31.0 mm outer width) + rounded tip nose (X_max=92.5)
      * 6.0 mm solid bottom rear bearing bridge (Z=[0, 6.0]) with coaxial Ø2.6 mm M3 pilot bore at (72.0, 0.0)
      * 4x Ø2.4 mm M3 pilot mounting holes (49.0 x 10.0 mm pitch) on top flange Z=34.0 (14 mm deep)
      * Spacious 11.0 x 12.0 mm bottom wire tunnel (X=[33.5, 41.5], Y=+/-5.5, Z=[6.0, 18.0]) under the rear
        M3 screw block -> opens into an open-top neck cable bay (X=[23.5, 33.5], Y=+/-5.5, Z>=6.0)
"""

import os
import math
from build123d import *

DIR = "/Users/manhngodh/development/robotics"

# --- Joint 1 (Bottom U-Yoke, mates to arm_base_v6_joint) ---
PIVOT_X = -6.0
PIVOT_Z = 24.10              # Coaxial midpoint of Z=[-1.0..49.2] MG996R #2 stack (with 30 mm pocket depth!)

MINUS_Y_IN = -19.5
MINUS_Y_OUT = -24.0          # 4.5 mm horn arm; MINUS_Y_IN=-19.5..PLUS_Y_IN=+37.0 -> 56.50 mm inner gap (>=56 mm!)
PLUS_Y_IN = 37.0
PLUS_Y_OUT = 42.0            # 5.0 mm arm: 1.0 mm lip (37.0..38.0) + 4.0 mm pocket (38.0..42.0) for 623ZZ (3x10x4 M3)
MINUS_ARM_R = 10.5           # Ø21 mm lug radius on -Y 25T horn arm
PLUS_ARM_R = 10.0            # Ø20 mm lug radius around Ø10.25 mm 623ZZ pocket (4.88 mm radial wall!)

BRIDGE_X_MIN = 17.5          # 23.5 mm throat from pivot (-6.0) -> >1 mm margin past R_corner=20.12 mm across +/-105 deg!
BRIDGE_X_MAX = 22.5          # 6.0 mm solid crossbar
BRIDGE_Z_MAX = 35.5          # Flush with HOUSING_TOP_Z=35.5 -> rock-solid straight top bridge & zero neck step!

# --- Joint 2 (Top MG996R #2: SHAFT & REAL WIRE at +X outer tip R=20.12 mm, side-exit slot at -Y!) ---
J2_SHAFT_X = 66.0            # 72.00 mm center-to-center; shaft near +X beveled tip (matches Joint Base V6!)
J2_SHAFT_Y = 0.5 * (MINUS_Y_IN + PLUS_Y_IN)  # +8.75 mm: centered symmetrically inside 56.50 mm U-yoke gap [-19.5, +37.0]!
J2_BODY_CX = J2_SHAFT_X - 10.35  # 55.65 mm (Body X=[35.30, 76.00], boot X=[30.30, 35.30], tabs X=[28.65, 82.65])

POCKET_X_MIN = 34.7          # 41.8 mm pocket length [34.7, 76.5] for 40.7 mm body + clearance
POCKET_LEN_X = 41.8
POCKET_WID_Y = 20.4          # 19.7 + 0.7 mm clearance -> Y=7.60 +/- 10.20
POCKET_FLOOR_Z = 5.5         # 5.5 mm bottom bearing bridge (Z=0..5.5)
HOUSING_TOP_Z = 35.5         # 30.0 mm pocket depth below mounting tabs (35.5 - 5.5 = 30.00 mm!)

HOUSING_X_MIN = 28.0         # -X screw & wire-boot block: X=[28.0, 34.7] (screws at X=31.15, boot tunnel below!)
HOUSING_HALF_Y = 15.5        # 31.0 mm outer width -> 5.3 mm solid side walls
NOSE_BEVEL_START_X = 76.5    # Start beveled trapezoid right at +X pocket end (matching Joint Base head)
NOSE_TIP_X = 84.0            # Beveled flat tip X=84.0 (18.0 mm from J2_SHAFT_X -> R_corner=20.12 mm!)
NOSE_TIP_HALF_Y = 9.0        # Bevels ±15.5 -> ±9.0 (tab ends at 82.65 where half-width=10.17 mm -> fits cleanly inside!)

MG996R_SCREW_X_OFF = 24.5    # 49.0 mm longitudinal pitch -> X = 45.85 & 94.85
MG996R_SCREW_Y_OFF = 5.0     # 10.0 mm transverse pitch -> Y = +/-5.0
MG996R_PILOT_R = 1.20        # Ø2.4 mm M3 pilot bore


def build_arm_link():
    def arm_solid(y0, y1, r_lug):
        thk = y1 - y0
        ymid = 0.5 * (y0 + y1)
        with BuildPart() as ap:
            with Locations((PIVOT_X, ymid, PIVOT_Z)):
                Cylinder(radius=r_lug, height=thk, rotation=(90, 0, 0))
            with BuildSketch(Plane.XZ.offset(-y1)) as sk:
                with BuildLine():
                    Polyline(
                        (8.0, 0.0),
                        (BRIDGE_X_MAX, 0.0),
                        (BRIDGE_X_MAX, BRIDGE_Z_MAX),
                        (PIVOT_X, PIVOT_Z + r_lug),
                        (PIVOT_X - r_lug, PIVOT_Z),
                        (PIVOT_X, PIVOT_Z - r_lug),
                        (8.0, 0.0),
                    )
                make_face()
            extrude(sk.sketch, amount=thk)
        return ap.part

    arm_minus = arm_solid(MINUS_Y_OUT, MINUS_Y_IN, MINUS_ARM_R)
    arm_plus = arm_solid(PLUS_Y_IN, PLUS_Y_OUT, PLUS_ARM_R)

    with BuildPart() as body:
        # 2. Crossbar bridge (X=[15.5, 23.5], Y=[-22.0, +36.7], Z=[0, 33.5])
        with Locations((0.5 * (BRIDGE_X_MIN + BRIDGE_X_MAX), 0.5 * (MINUS_Y_OUT + PLUS_Y_OUT), BRIDGE_Z_MAX / 2.0)):
            Box(BRIDGE_X_MAX - BRIDGE_X_MIN, PLUS_Y_OUT - MINUS_Y_OUT, BRIDGE_Z_MAX)

        # 3. Symmetric tapered shoulder transition (X=[23.0, 35.0], Y=[-22.0..+36.7 -> 6.6+/-15.5], Z=[0, 33.5])
        with BuildSketch(Plane.XY) as sk_taper:
            with BuildLine():
                Polyline(
                    (BRIDGE_X_MAX - 0.5, MINUS_Y_OUT),
                    (HOUSING_X_MIN + 1.5, J2_SHAFT_Y - HOUSING_HALF_Y),
                    (HOUSING_X_MIN + 1.5, J2_SHAFT_Y + HOUSING_HALF_Y),
                    (BRIDGE_X_MAX - 0.5, PLUS_Y_OUT),
                    (BRIDGE_X_MAX - 0.5, MINUS_Y_OUT),
                )
            make_face()
        extrude(sk_taper.sketch, amount=BRIDGE_Z_MAX)

        # 5. Monolithic MG996R #2 upper housing (X=[33.5, 82.6] + beveled flat trapezoid tip to X=90.0 matching Joint Base!)
        with Locations((0.5 * (HOUSING_X_MIN + NOSE_BEVEL_START_X), J2_SHAFT_Y, HOUSING_TOP_Z / 2.0)):
            Box(NOSE_BEVEL_START_X - HOUSING_X_MIN, HOUSING_HALF_Y * 2.0, HOUSING_TOP_Z)
        with BuildSketch(Plane.XY) as sk_nose:
            with BuildLine():
                Polyline(
                    (NOSE_BEVEL_START_X - 0.1, J2_SHAFT_Y - HOUSING_HALF_Y),
                    (NOSE_TIP_X, J2_SHAFT_Y - NOSE_TIP_HALF_Y),
                    (NOSE_TIP_X, J2_SHAFT_Y + NOSE_TIP_HALF_Y),
                    (NOSE_BEVEL_START_X - 0.1, J2_SHAFT_Y + HOUSING_HALF_Y),
                    (NOSE_BEVEL_START_X - 0.1, J2_SHAFT_Y - HOUSING_HALF_Y),
                )
            make_face()
        extrude(sk_nose.sketch, amount=HOUSING_TOP_Z)

    part = arm_minus + arm_plus + body.part

    # --- SUBTRACTIONS ---
    with BuildPart() as cuts:
        # A. Joint 1 -Y 25T round plastic horn pattern (Ø6.2 center + 4x Ø2.20 mm on Ø14.0 PCD + Ø4.6x1.5 outer seat)
        with Locations((PIVOT_X, 0.5 * (MINUS_Y_OUT + MINUS_Y_IN), PIVOT_Z)):
            Cylinder(radius=3.1, height=7.0, rotation=(90, 0, 0))
        for k in range(4):
            ang = math.radians(45.0 + k * 90.0)
            hx = PIVOT_X + 7.0 * math.cos(ang)
            hz = PIVOT_Z + 7.0 * math.sin(ang)
            with Locations((hx, 0.5 * (MINUS_Y_OUT + MINUS_Y_IN), hz)):
                Cylinder(radius=1.10, height=7.0, rotation=(90, 0, 0))
            with Locations((hx, MINUS_Y_OUT + 0.5, hz)):
                Cylinder(radius=2.30, height=2.0, rotation=(90, 0, 0))

        # B. Joint 1 +Y 623ZZ (3x10x4 M3) bearing pocket (Y=38.0..42.0 -> Ø10.25) & inner lip (Y=37.0..38.0 -> Ø7.0)
        with Locations((PIVOT_X, 0.5 * (PLUS_Y_IN + PLUS_Y_OUT), PIVOT_Z)):
            Cylinder(radius=3.50, height=8.0, rotation=(90, 0, 0))
        with Locations((PIVOT_X, 0.5 * ((PLUS_Y_IN + 1.0) + PLUS_Y_OUT + 2.0), PIVOT_Z)):
            Cylinder(radius=5.125, height=(PLUS_Y_OUT - (PLUS_Y_IN + 1.0)) + 2.0, rotation=(90, 0, 0))

        # C. Clean rectangular top-open MG996R #2 servo pocket (X=[34.70, 76.50], Y=7.60+/-10.20, Z=[5.5, 36.5])
        with Locations((0.5 * (POCKET_X_MIN + NOSE_BEVEL_START_X), J2_SHAFT_Y, 0.5 * (POCKET_FLOOR_Z + HOUSING_TOP_Z + 1.0))):
            Box(NOSE_BEVEL_START_X - POCKET_X_MIN, POCKET_WID_Y, (HOUSING_TOP_Z + 1.0) - POCKET_FLOOR_Z)

        # D. 4x M3 pilot mounting bores on top flange Z=35.5 (49.0 x 10.0 mm pitch, 11.7 mm deep -> Z=23.8..35.5)
        for sx_off in (-MG996R_SCREW_X_OFF, MG996R_SCREW_X_OFF):
            for sy_off in (-MG996R_SCREW_Y_OFF, MG996R_SCREW_Y_OFF):
                with Locations((J2_BODY_CX + sx_off, J2_SHAFT_Y + sy_off, HOUSING_TOP_Z - 5.75)):
                    Cylinder(radius=MG996R_PILOT_R, height=11.7)

        # E. Wire & boot bay matching Joint Base V6 (#9 layout, 1-to-1 mirrored onto Join Link +X shaft end):
        #    - Depth from top flange Z=35.5: starts 13.0 mm down (Z<=22.5, leaves 1.3 mm under 11.7 mm +X M3 screws!)
        #      down to POCKET_FLOOR_Z=5.5 (keeps 5.5 mm bottom bearing bridge intact).
        #    - Opens out -Y side wall (Y=-7.25) and +X tip (X=84.0), extending +6.5 mm past J2_SHAFT_Y (+15.25),
        #      leaving a 9.0 mm solid +Y beam (+15.25..+24.25) supporting the top M3 flange!
        bay_z_min = POCKET_FLOOR_Z          # 5.50 mm
        bay_z_max = HOUSING_TOP_Z - 13.0    # 22.50 mm (1.30 mm below M3 bore bottoms at Z=23.8)
        bay_x_min = NOSE_BEVEL_START_X - 1.8  # 74.70 mm
        bay_x_max = NOSE_TIP_X + 1.0        # 85.00 mm
        bay_y_min = J2_SHAFT_Y - 16.0       # -7.25 mm (opens cleanly past -6.75 outer -Y wall)
        bay_y_max = J2_SHAFT_Y + 6.5        # +15.25 mm (keeps +15.25..+24.25 = 9.0 mm +Y beam solid)
        with Locations((0.5 * (bay_x_min + bay_x_max), 0.5 * (bay_y_min + bay_y_max), 0.5 * (bay_z_min + bay_z_max))):
            Box(bay_x_max - bay_x_min, bay_y_max - bay_y_min, bay_z_max - bay_z_min)

        # F. Coaxial M3 pilot bore in Z=0..5.5 bottom bearing bridge at (66.0, 8.75) for 623ZZ (3x10x4)
        with Locations((J2_SHAFT_X, J2_SHAFT_Y, 2.4)):
            Cylinder(radius=1.30, height=5.0)

    part = part - cuts.part

    try:
        part = part.clean()
    except Exception:
        pass
    return part


def place_in_world(link, shaft_z=67.0, angle_deg=0.0):
    """Place link onto arm_base_v6_joint in world frame (Joint 1 axis along X_w at Y=0, Z=shaft_z)."""
    p = link.translate((-PIVOT_X, 0.0, -PIVOT_Z))
    p = p.rotate(Axis.Y, -90).rotate(Axis.Z, -90)
    p = p.translate((-15.0, 0.0, shaft_z))
    if abs(angle_deg) > 1e-6:
        p = p.rotate(Axis((-15.0, 0.0, shaft_z), (1, 0, 0)), angle_deg)
    return p


def build_virtual_mg996r2():
    """Virtual MG996R #2 seated on HOUSING_TOP_Z=35.5 (shaft at +X=66.0, real wire exits +X=76.0 into V6-style bay)."""
    z0 = HOUSING_TOP_Z - 27.5
    with BuildPart() as m:
        with Locations((J2_BODY_CX, J2_SHAFT_Y, z0 + 27.5 / 2.0)):
            Box(40.7, 19.7, 27.5)
        with Locations((J2_BODY_CX, J2_SHAFT_Y, HOUSING_TOP_Z + 11.2 / 2.0)):
            Box(40.7, 19.7, 11.2)
        with Locations((J2_BODY_CX, J2_SHAFT_Y, HOUSING_TOP_Z + 1.3)):
            Box(54.0, 19.7, 2.6)
        with Locations((J2_SHAFT_X, J2_SHAFT_Y, HOUSING_TOP_Z + 11.2 + 2.4)):
            Cylinder(radius=2.95, height=4.8)
        # Real MG996R 3-wire ribbon exit: emerges at +X=76.0 (Z=9.5..13.5), bends -Y out bay (0..105 deg safe)
        with Locations((77.5, J2_SHAFT_Y - 5.5, 11.5)):
            Box(3.5, 11.0, 3.0)
    return m.part


def main():
    print("Building monolithic MG996R-to-MG996R arm link...")
    link = build_arm_link()
    bb = link.bounding_box()
    print(f"  local bbox: X=[{bb.min.X:.2f},{bb.max.X:.2f}] ({bb.max.X-bb.min.X:.2f}), "
          f"Y=[{bb.min.Y:.2f},{bb.max.Y:.2f}] ({bb.max.Y-bb.min.Y:.2f}), "
          f"Z=[{bb.min.Z:.2f},{bb.max.Z:.2f}] ({bb.max.Z-bb.min.Z:.2f})")
    print(f"  link length (J1->J2) = {J2_SHAFT_X - PIVOT_X:.2f} mm")
    print(f"  is_valid={link.is_valid}, solids={len(link.solids())}, vol={link.volume:.1f} mm3 (~{link.volume*0.00127:.1f} g @100%)")

    step_path = os.path.join(DIR, "arm_link_mg996r_to_mg996r.step")
    stl_path = os.path.join(DIR, "arm_link_mg996r_to_mg996r.stl")
    export_step(link, step_path)
    export_stl(link, stl_path)

    # Print-ready STL centered in XY with Z_min = 0.0
    pr = link.translate((-0.5 * (bb.min.X + bb.max.X), -0.5 * (bb.min.Y + bb.max.Y), -bb.min.Z))
    pr_path = os.path.join(DIR, "PrintReady_MG996R_to_MG996R.stl")
    export_stl(pr, pr_path)

    # Collision check against arm_base_v6_joint + MG996R #1 across swing angles
    from build_arm_base_v6 import SHAFT_Z, BODY_CENTER_Z
    joint = import_step(os.path.join(DIR, "arm_base_v6_joint.step"))
    with BuildPart() as mg1:
        with Locations((-18.5 + 9.0, 0, BODY_CENTER_Z)):
            Box(27.5, 19.7, 40.7)
        with Locations((-18.5 - 5.6, 0, BODY_CENTER_Z)):
            Box(11.2, 19.7, 40.7)
        with Locations((-18.5 - 1.3, 0, BODY_CENTER_Z)):
            Box(2.6, 19.7, 54.0)
        with Locations((-18.5 - 13.6, 0, SHAFT_Z)):
            Cylinder(radius=2.95, height=4.8, rotation=(0, 90, 0))

    print(f"\nCollision sweep against arm_base_v6_joint (SHAFT_Z={SHAFT_Z}):")
    for ang in [0.0, 45.0, 90.0, 105.0, -45.0, -90.0, -105.0]:
        lw = place_in_world(link, shaft_z=SHAFT_Z, angle_deg=ang)
        ij = lw.intersect(joint)
        vj = sum(s.volume for s in ij.solids()) if (ij is not None and len(ij) > 0) else 0.0
        im = lw.intersect(mg1.part)
        vm = sum(s.volume for s in im.solids()) if (im is not None and len(im) > 0) else 0.0
        print(f"  angle={ang:+6.1f} deg -> joint overlap={vj:.3f} mm3 | mg996r_1 overlap={vm:.3f} mm3")

    # Virtual MG996R #2 drop-in clearance check
    mg2_loc = build_virtual_mg996r2()
    i2 = link.intersect(mg2_loc)
    v2 = sum(s.volume for s in i2.solids()) if (i2 is not None and len(i2) > 0) else 0.0
    print(f"MG996R #2 pocket fit overlap = {v2:.3f} mm3 (expected 0.000)")

    # Export full mating assembly STEP
    lw0 = place_in_world(link, shaft_z=SHAFT_Z, angle_deg=0.0)
    mg2_w = place_in_world(mg2_loc, shaft_z=SHAFT_Z, angle_deg=0.0)
    asm = Compound(children=[joint, mg1.part, lw0, mg2_w])
    asm_path = os.path.join(DIR, "arm_mg996r_to_mg996r_assembly_check.step")
    export_step(asm, asm_path)
    print(f"Saved {step_path}, {pr_path}, and {asm_path}")


if __name__ == "__main__":
    main()

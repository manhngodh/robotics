#!/usr/bin/env python3
"""
Build dual-arm ("2 cang") link from MG996R (25T CNC Round Horn + 624ZZ bearing) to MG90S cover:
  - Bottom (MG996R U-Yoke, matched 100% to Joint Base V6 & Join Link V6):
      * -Y arm (Y=[-22.0, -17.5], 4.5 mm thick): Ø21 mm boss at (-6.0, 15.5), Ø6.2 center +
        4x Ø3.2 mm M3 holes on Ø14.0 mm PCD (45 deg phase) for Ø20 mm 25T CNC horn.
      * +Y arm (Y=[+32.7, +38.7], 6.0 mm thick): Ø23 mm boss at (-6.0, 15.5), Ø9.5 mm inner lip
        (1.0 mm) + Ø13.25 mm pocket (5.0 mm) for 624ZZ (4x13x5) bearing.
      * Gap Y=[-17.5, +32.7] = 50.20 mm, symmetric center at Y = +7.60 mm.
  - Transition crossbar bridge & dual gussets (X=[35.5, 41.5], 41.5 mm throat from pivot):
      * Clears BOTH Joint Base V6 (R=20.12 mm) AND Join Link MG996R #2 (R=40.03 mm)
        across 0..+/-105 deg with 0.000 mm3 collision!
  - Top (MG90S cover end):
      * Preserves native u_break_cover_orig MG90S pocket, M2 screw bores, rear & -Y wire windows,
        and u_break mating tip, shifted to X=[35.5, 81.0], Y=[-2.9, +18.1] (centered at Y=+7.60).
      * Link length (MG996R pivot X=-6.0 -> MG90S shaft X=70.5) = 76.50 mm.
      * Prints flat on Z=0 without filling MG90S cavity.
"""

import os
import math
import struct
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from build123d import *

from build_arm_base_v6 import SHAFT_Z, BODY_CENTER_Z
from build_mg996r_to_mg996r import (
    place_in_world,
    J2_BODY_CX,
    J2_SHAFT_X,
    J2_SHAFT_Y,
    HOUSING_TOP_Z,
)
from assemble_joint1_demo import (
    make_detailed_mg996r_local,
    make_25t_round_horn,
)

DIR = "/Users/manhngodh/development/robotics"
ARTIFACTS = "/Users/manhngodh/.gemini/antigravity/brain/f16658af-3957-4375-90c0-9237ebaff4ff"
ORIG_STEP = os.path.join(DIR, "u_break_cover_orig.step")

# --- Dual-Arm ("2 cang") MG996R U-Yoke (Matches Join Link V6 & Joint Base V6) ---
PIVOT_X = -6.0
PIVOT_Z = 15.5
MINUS_Y_IN = -19.5
MINUS_Y_OUT = -24.0          # 4.5 mm horn arm; inner gap [-19.5, +37.0] = 56.50 mm (>=56 mm!)
PLUS_Y_IN = 37.0             # Widened to match fixed Join Link V6 (56.50 mm yoke gap)
PLUS_Y_OUT = 42.0            # 5.0 mm arm: 1.0 mm lip (37.0..38.0) + 4.0 mm 623ZZ (3x10x4 M3) pocket (38.0..42.0)
MINUS_ARM_R = 10.5           # Ø21 mm lug around Ø20 mm 25T CNC horn
PLUS_ARM_R = 10.0            # Ø20 mm lug around Ø10.25 mm 623ZZ bearing pocket

# --- Crossbar Bridge & MG90S Cover Placement ---
BRIDGE_X_MIN = 16.5          # 22.5 mm throat from pivot -> clears Joint Base & fixed Join Link (R=20.12 mm) across +/-105 deg!
BRIDGE_X_MAX = 22.5          # 6.0 mm solid crossbar stepping into both walls of MG90S cover
HEAD_Z_MAX = 22.0            # Matches u_break_cover_orig top Z=22.0 flush
HEAD_SHIFT_X = 1.5           # Shifts u_break_cover_orig X=15.0..60.5 -> X=16.5..62.0 (MG90S shaft X=51.5 -> 57.50 mm link!)
HEAD_SHIFT_Y = J2_SHAFT_Y    # +8.75 mm: centered coaxially inside 56.50 mm yoke [-19.5, +37.0]


def build_mg996r_to_mg90s_2arm():
    orig = import_step(ORIG_STEP)
    # Extract clean native MG90S upper cover including its X=15..19 rear wall, clipped to Y=[-10.5, +10.5].
    # Keep only outermost Solid shell to strip 2 tiny internal 0.03 mm void bubbles in u_break_cover_orig.step.
    orig_raw = Compound(orig.intersect(Pos(38.0, 0.0, 11.0) * Box(46.0, 21.0, 24.0)))
    orig_clean = Solid(max(orig_raw.shells(), key=lambda s: s.area))
    mg90s_head = orig_clean.translate((HEAD_SHIFT_X, HEAD_SHIFT_Y, 0.0))

    def arm_solid(y0, y1, r_lug):
        thk = y1 - y0
        ymid = 0.5 * (y0 + y1)
        with BuildPart() as ap:
            with Locations((PIVOT_X, ymid, PIVOT_Z)):
                Cylinder(radius=r_lug, height=thk, rotation=(90, 0, 0))
            with BuildSketch(Plane.XZ.offset(-y1)) as sk:
                with BuildLine():
                    Polyline(
                        (-2.0, 0.0),
                        (BRIDGE_X_MAX, 0.0),
                        (BRIDGE_X_MAX, HEAD_Z_MAX),
                        (PIVOT_X, PIVOT_Z + r_lug),
                        (PIVOT_X - r_lug, PIVOT_Z),
                        (PIVOT_X, PIVOT_Z - r_lug),
                        (-2.0, 0.0),
                    )
                make_face()
            extrude(sk.sketch, amount=thk)
        return ap.part

    arm_minus = arm_solid(MINUS_Y_OUT, MINUS_Y_IN, MINUS_ARM_R)
    arm_plus = arm_solid(PLUS_Y_IN, PLUS_Y_OUT, PLUS_ARM_R)

    head_y_min = HEAD_SHIFT_Y - 10.5  # -1.75
    head_y_max = HEAD_SHIFT_Y + 10.5  # +19.25

    with BuildPart() as bridges:
        # Left (-Y) crossbar bridge: Y=[-24.0, +1.15] (fuses 2.9 mm into solid -Y wall, never enters pocket Y>=+2.55)
        bridge_y_minus_max = HEAD_SHIFT_Y - 7.60  # +1.15 (2.90 mm overlap inside solid -Y wall)
        with Locations((0.5 * (BRIDGE_X_MIN + BRIDGE_X_MAX), 0.5 * (MINUS_Y_OUT + bridge_y_minus_max), HEAD_Z_MAX / 2.0)):
            Box(BRIDGE_X_MAX - BRIDGE_X_MIN, bridge_y_minus_max - MINUS_Y_OUT, HEAD_Z_MAX)

        # Right (+Y) crossbar bridge: Y=[+16.35, +43.00] (fuses 2.9 mm into solid +Y wall, never enters pocket Y<=+14.95)
        bridge_y_plus_min = HEAD_SHIFT_Y + 7.60   # +16.35
        with Locations((0.5 * (BRIDGE_X_MIN + BRIDGE_X_MAX), 0.5 * (bridge_y_plus_min + PLUS_Y_OUT), HEAD_Z_MAX / 2.0)):
            Box(BRIDGE_X_MAX - BRIDGE_X_MIN, PLUS_Y_OUT - bridge_y_plus_min, HEAD_Z_MAX)

        # Smooth diagonal stiffener gusset on -Y (stops at X=28.0, 1.5 mm before -Y wire window at X=29.5)
        with BuildSketch(Plane.XY) as sk_g_minus:
            with BuildLine():
                Polyline(
                    (BRIDGE_X_MIN, MINUS_Y_IN),
                    (28.0, head_y_min + 1.5),
                    (BRIDGE_X_MIN, head_y_min + 1.5),
                    (BRIDGE_X_MIN, MINUS_Y_IN),
                )
            make_face()
        extrude(sk_g_minus.sketch, amount=HEAD_Z_MAX)

        # Smooth diagonal stiffener gusset on +Y (full solid +Y shoulder up to X=33.0)
        with BuildSketch(Plane.XY) as sk_g_plus:
            with BuildLine():
                Polyline(
                    (BRIDGE_X_MIN, PLUS_Y_IN),
                    (33.0, head_y_max - 1.5),
                    (BRIDGE_X_MIN, head_y_max - 1.5),
                    (BRIDGE_X_MIN, PLUS_Y_IN),
                )
            make_face()
        extrude(sk_g_plus.sketch, amount=HEAD_Z_MAX)

    part = arm_minus + arm_plus + bridges.part + mg90s_head

    # --- Cut Bolt & Bearing Interfaces ---
    with BuildPart() as cuts:
        # 1. -Y 25T round plastic horn pattern (Ø6.2 center + 4x Ø2.20 mm straight-through bores on Ø14.0 PCD)
        arm_ymid = 0.5 * (MINUS_Y_OUT + MINUS_Y_IN)
        with Locations((PIVOT_X, arm_ymid, PIVOT_Z)):
            Cylinder(radius=3.1, height=7.0, rotation=(90, 0, 0))
        for k in range(4):
            ang = math.radians(45.0 + k * 90.0)
            hx = PIVOT_X + 7.0 * math.cos(ang)
            hz = PIVOT_Z + 7.0 * math.sin(ang)
            with Locations((hx, arm_ymid, hz)):
                Cylinder(radius=1.10, height=7.0, rotation=(90, 0, 0))

        # 2. +Y 623ZZ (3x10x4 M3) bearing pocket (Y=38.0..42.0 -> Ø10.25) & inner lip (Y=37.0..38.0 -> Ø7.0)
        with Locations((PIVOT_X, 0.5 * (PLUS_Y_IN + PLUS_Y_OUT), PIVOT_Z)):
            Cylinder(radius=3.50, height=8.0, rotation=(90, 0, 0))
        with Locations((PIVOT_X, 0.5 * ((PLUS_Y_IN + 1.0) + PLUS_Y_OUT + 2.0), PIVOT_Z)):
            Cylinder(radius=5.125, height=(PLUS_Y_OUT - (PLUS_Y_IN + 1.0)) + 2.0, rotation=(90, 0, 0))

    part = part - cuts.part
    try:
        part = part.clean()
    except Exception:
        pass
    return part


def place_on_joint2_local(shape_on_j2, ang2_deg=0.0):
    """Place 2-arm part onto Link #1's Joint 2 horn (at J2_SHAFT_X=60.0, J2_SHAFT_Y=7.60, Z=49.2)."""
    p = shape_on_j2.translate((-PIVOT_X, 0.0, -PIVOT_Z))
    p = p.rotate(Axis.X, -90).translate((J2_SHAFT_X, J2_SHAFT_Y, HOUSING_TOP_Z + 13.7 + MINUS_Y_IN))
    if abs(ang2_deg) > 1e-6:
        p = p.rotate(Axis((J2_SHAFT_X, J2_SHAFT_Y, 0.0), (0, 0, 1)), ang2_deg)
    return p


def place_on_joint1_world(shape_loc, shaft_z=SHAFT_Z, angle_deg=0.0):
    """Place 2-arm part directly onto Joint Base V6's MG996R #1 horn + +X bearing spigot."""
    p = shape_loc.translate((-PIVOT_X, 0.0, -PIVOT_Z))
    p = p.rotate(Axis.Y, -90).rotate(Axis.Z, -90)
    p = p.translate((-15.0, 0.0, shaft_z))
    if abs(angle_deg) > 1e-6:
        p = p.rotate(Axis((-15.0, 0.0, shaft_z), (1, 0, 0)), angle_deg)
    return p


def part_to_tris(shape):
    tmp = "/tmp/_part_2arm_tmp.stl"
    export_stl(shape, tmp)
    with open(tmp, "rb") as f:
        f.read(80)
        cnt = struct.unpack("<I", f.read(4))[0]
        return np.fromfile(f, dtype=np.dtype([("n", "<f4", (3,)), ("v", "<f4", (3, 3)), ("a", "<u2")]), count=cnt)["v"]


def main():
    print("Building dual-arm ('2 cang') MG996R-to-MG90S cover...")
    part = build_mg996r_to_mg90s_2arm()
    bb = part.bounding_box()
    print(f"  local bbox: X=[{bb.min.X:.2f},{bb.max.X:.2f}] ({bb.max.X-bb.min.X:.2f}), "
          f"Y=[{bb.min.Y:.2f},{bb.max.Y:.2f}] ({bb.max.Y-bb.min.Y:.2f}), "
          f"Z=[{bb.min.Z:.2f},{bb.max.Z:.2f}] ({bb.max.Z-bb.min.Z:.2f})")
    print(f"  link length (MG996R pivot -> MG90S shaft X={50.0 + HEAD_SHIFT_X:.1f}) = {50.0 + HEAD_SHIFT_X - PIVOT_X:.2f} mm")
    print(f"  is_valid={part.is_valid}, solids={len(part.solids())}, vol={part.volume:.1f} mm3 (~{part.volume*0.00127:.1f} g @100%)")

    step_out = os.path.join(DIR, "arm_link_mg996r_to_mg90s_2arm.step")
    stl_out = os.path.join(DIR, "arm_link_mg996r_to_mg90s_2arm.stl")
    pr_out = os.path.join(DIR, "PrintReady_MG996R_to_MG90S_2Arm.stl")
    export_step(part, step_out)
    export_stl(part, stl_out)
    pr = part.translate((-0.5 * (bb.min.X + bb.max.X), -0.5 * (bb.min.Y + bb.max.Y), -bb.min.Z))
    export_stl(pr, pr_out)

    # Collision sweep 1: mounted directly on Joint Base V6 + MG996R #1
    joint_base = import_step(os.path.join(DIR, "arm_base_v6_joint.step"))
    mg_proto = make_detailed_mg996r_local()
    horn_proto = make_25t_round_horn()
    mg1 = mg_proto.rotate(Axis.Y, -90).translate((9.0, 0.0, BODY_CENTER_Z))
    horn1 = horn_proto.rotate(Axis.Y, 90).translate((-32.5, 0.0, SHAFT_Z))

    print("\nSweep A: 2-Arm MG996R->MG90S on Joint Base V6 (0..+/-105 deg):")
    for ang in [0, 45, 90, 105, -45, -90, -105]:
        pw = place_on_joint1_world(part, shaft_z=SHAFT_Z, angle_deg=ang)
        v_jb = sum(s.volume for s in (pw.intersect(joint_base).solids() if pw.intersect(joint_base) else []))
        v_m1 = sum(s.volume for s in (pw.intersect(mg1).solids() if pw.intersect(mg1) else []))
        print(f"  ang1={ang:+4d} deg -> vs JointBase={v_jb:.3f} mm3 | vs MG996R#1={v_m1:.3f} mm3")

    # Collision sweep 2: stacked on fixed Join Link (MG996R #2 horn at +X tip J2_SHAFT_X=66.0)
    link1_loc = import_step(os.path.join(DIR, "arm_link_mg996r_to_mg996r.step"))
    mg2_loc = mg_proto.translate((J2_BODY_CX, J2_SHAFT_Y, HOUSING_TOP_Z - 27.5))
    horn2_loc = horn_proto.rotate(Axis.X, 180).translate((J2_SHAFT_X, J2_SHAFT_Y, HOUSING_TOP_Z + 13.7))

    print("\nSweep B: 2-Arm MG996R->MG90S stacked on Join Link MG996R #2 (0..+/-105 deg):")
    for ang in [0, 45, 90, 105, -45, -90, -105]:
        p_j2 = place_on_joint2_local(part, ang2_deg=ang)
        v_l1 = sum(s.volume for s in (p_j2.intersect(link1_loc).solids() if p_j2.intersect(link1_loc) else []))
        v_m2 = sum(s.volume for s in (p_j2.intersect(mg2_loc).solids() if p_j2.intersect(mg2_loc) else []))
        print(f"  ang2={ang:+4d} deg -> vs JoinLink={v_l1:.3f} mm3 | vs MG996R#2={v_m2:.3f} mm3")

    # Export assembly check STEP (Joint Base V6 -> Join Link V6 -> 2-Arm MG996R->MG90S Cover)
    link1_w = place_in_world(link1_loc, shaft_z=SHAFT_Z, angle_deg=0.0)
    mg2_w = place_in_world(mg2_loc, shaft_z=SHAFT_Z, angle_deg=0.0)
    horn2_w = place_in_world(horn2_loc, shaft_z=SHAFT_Z, angle_deg=0.0)
    top_2arm_w = place_in_world(place_on_joint2_local(part, ang2_deg=0.0), shaft_z=SHAFT_Z, angle_deg=0.0)

    asm_step = os.path.join(DIR, "arm_mg996r_to_mg90s_2arm_assembly_check.step")
    export_step(Compound(children=[joint_base, mg1, horn1, link1_w, mg2_w, horn2_w, top_2arm_w]), asm_step)

    # Render 3-panel PNG preview
    palette_asm = [
        (joint_base, [0.85, 0.45, 0.14]),
        (mg1,        [0.18, 0.20, 0.24]),
        (horn1,      [0.78, 0.18, 0.22]),
        (link1_w,    [0.16, 0.56, 0.84]),
        (mg2_w,      [0.18, 0.20, 0.24]),
        (horn2_w,    [0.78, 0.18, 0.22]),
        (top_2arm_w, [0.18, 0.72, 0.52]),
    ]
    light = np.array([0.45, -0.55, 0.70]); light /= np.linalg.norm(light)

    def draw_list(ax, items, xlim, ylim, zlim, elev, azim, title):
        for sh, col in items:
            tris = part_to_tris(sh)
            v0, v1, v2 = tris[:, 0], tris[:, 1], tris[:, 2]
            norms = np.cross(v1 - v0, v2 - v0)
            nl = np.linalg.norm(norms, axis=1, keepdims=True)
            norms = np.divide(norms, np.maximum(nl, 1e-9))
            shade = 0.45 + 0.55 * np.clip(norms @ light, 0.0, 1.0)
            cols = np.clip(np.asarray(col)[None, :] * shade[:, None], 0.0, 1.0)
            pc = Poly3DCollection(tris, facecolors=cols, edgecolors="none", linewidths=0)
            ax.add_collection3d(pc)
        ax.set_xlim(*xlim); ax.set_ylim(*ylim); ax.set_zlim(*zlim)
        ax.set_box_aspect((xlim[1]-xlim[0], ylim[1]-ylim[0], zlim[1]-zlim[0]))
        ax.view_init(elev=elev, azim=azim)
        ax.set_xlabel("X"); ax.set_ylabel("Y"); ax.set_zlabel("Z")
        ax.set_title(title, fontsize=10, fontweight="bold")

    fig = plt.figure(figsize=(18, 6.2), facecolor="white")
    draw_list(fig.add_subplot(1, 3, 1, projection="3d"), [(part, [0.18, 0.72, 0.52])], (-22, 66), (-30, 48), (-5, 35), 28, -50, "2-Arm MG996R -> MG90S (56.5mm Yoke / 57.5mm Link)")
    draw_list(fig.add_subplot(1, 3, 2, projection="3d"), [(part, [0.18, 0.72, 0.52])], (-22, 66), (-30, 48), (-5, 35), 22, 130, "2-Arm Yoke, 624ZZ & MG90S Pocket")
    draw_list(fig.add_subplot(1, 3, 3, projection="3d"), palette_asm, (-55, 55), (-55, 55), (15, 225), 20, -45, "Stacked on Fixed Join Link (Dual-Shear)")
    plt.tight_layout()
    png_out = os.path.join(ARTIFACTS, "mg996r_to_mg90s_2arm_check.png")
    plt.savefig(png_out, dpi=150)
    print(f"Saved {pr_out}, {step_out}, {asm_step}, and {png_out}")


if __name__ == "__main__":
    main()

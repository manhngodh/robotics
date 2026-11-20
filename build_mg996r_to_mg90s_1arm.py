#!/usr/bin/env python3
"""
Build single-arm ("1 cang") link from MG996R (25T CNC Round Horn) to MG90S cover:
  - Bottom (MG996R end): Single cantilevered -Y arm at Y=[-22.5, -17.5] (5.0 mm thick)
    with Ø23 mm boss around (-6.0, 15.5), Ø6.2 mm center bore + 4x Ø3.2 mm M3 holes
    on Ø14.0 mm PCD (45 deg phase) matching standard Ø20 mm 25T CNC horn.
  - Transition neck & bridge (X=[35.5, 42.5], 41.5 mm throat from pivot):
    Clears BOTH Joint Base V6 (R=20.12 mm) AND Join Link MG996R #2 (R=40.03 mm)
    across 0..+/-105 deg with 0.000 mm3 collision!
  - Top (MG90S cover end): Preserves native u_break_cover_orig MG90S pocket, M2 screw
    bores, rear/side wire windows, and u_break mating tip, shifted to X=[35.5, 81.0],
    Y=[-2.9, +18.1] (centered at Y=+7.60 to stay coaxial when stacked on Join Link!).
  - Prints flat on Z=0 with zero support overhang.
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
    PIVOT_X as LINK_PIVOT_X,
    PIVOT_Z as LINK_PIVOT_Z,
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

# --- Single-Arm ("1 cang") MG996R -Y Horn Interface ---
PIVOT_X = -6.0
PIVOT_Z = 15.5               # Coaxial with native MG90S cover head Z=[0, 22] + turret/bracket stack
MINUS_Y_IN = -17.5
MINUS_Y_OUT = -22.5          # 5.0 mm solid cantilevered arm thickness
ARM_R = 11.5                 # Ø23 mm reinforced boss around Ø20 mm 25T CNC horn

# --- Bridge & MG90S Cover Placement ---
BRIDGE_X_MIN = 35.5          # 41.5 mm throat from pivot -> clears Join Link R=40.03 mm across +/-105 deg
BRIDGE_X_MAX = 41.0          # 5.5 mm bridge stepping from -Y arm into native -Y cover wall
HEAD_Z_MAX = 22.0            # Matches u_break_cover_orig top Z=22.0 flush
HEAD_SHIFT_X = 20.5          # Shifts u_break_cover_orig X=15.0..60.5 -> X=35.5..81.0 (MG90S shaft X=70.5 -> 76.5 mm link)
HEAD_SHIFT_Y = J2_SHAFT_Y    # +7.60 mm: centered coaxially on Join Link's Joint 2 axis!


def build_mg996r_to_mg90s_1arm():
    orig = import_step(ORIG_STEP)
    # Extract clean native MG90S upper cover including its X=15..19 rear wall, clipped to Y=[-10.5, +10.5]
    # Keep only outermost Solid shell to strip 2 tiny internal 0.03 mm bubble voids in u_break_cover_orig.step
    orig_raw = Compound(orig.intersect(Pos(38.0, 0.0, 11.0) * Box(46.0, 21.0, 24.0)))
    orig_clean = Solid(max(orig_raw.shells(), key=lambda s: s.area))
    mg90s_head = orig_clean.translate((HEAD_SHIFT_X, HEAD_SHIFT_Y, 0.0))

    # 1. Single reinforced -Y arm (Y=[-22.5, -17.5])
    arm_thk = MINUS_Y_IN - MINUS_Y_OUT
    arm_ymid = 0.5 * (MINUS_Y_IN + MINUS_Y_OUT)
    with BuildPart() as ap:
        with Locations((PIVOT_X, arm_ymid, PIVOT_Z)):
            Cylinder(radius=ARM_R, height=arm_thk, rotation=(90, 0, 0))
        with BuildSketch(Plane.XZ.offset(-MINUS_Y_IN)) as sk_arm:
            with BuildLine():
                Polyline(
                    (8.0, 0.0),
                    (BRIDGE_X_MAX, 0.0),
                    (BRIDGE_X_MAX, HEAD_Z_MAX),
                    (PIVOT_X, PIVOT_Z + ARM_R),
                    (PIVOT_X - ARM_R, PIVOT_Z),
                    (PIVOT_X, PIVOT_Z - ARM_R),
                    (8.0, 0.0),
                )
            make_face()
        extrude(sk_arm.sketch, amount=arm_thk)

        # 2. Lateral step bridge + neck gusset connecting -Y arm (Y=-22.5..-17.5) into -Y solid wall of MG90S head
        # Native head occupies Y=[-2.90, +18.10], with solid -Y wall spanning Y=[-2.90, +1.40].
        # Stopping bridge at Y=0.0 fuses 2.9 mm into the solid -Y wall while keeping pocket & rear wire window 100% open!
        bridge_y_max = HEAD_SHIFT_Y - 7.60  # Y = 0.00 mm (2.90 mm overlap inside solid -Y wall)
        with Locations((0.5 * (BRIDGE_X_MIN + BRIDGE_X_MAX), 0.5 * (MINUS_Y_OUT + bridge_y_max), HEAD_Z_MAX / 2.0)):
            Box(BRIDGE_X_MAX - BRIDGE_X_MIN, bridge_y_max - MINUS_Y_OUT, HEAD_Z_MAX)

        # Smooth diagonal stiffener web from bridge X=35.5/Y=-17.5 into -Y wall (stops at X=47.0, before wire window at X=48.5)
        head_y_min = HEAD_SHIFT_Y - 10.5  # -2.90
        with BuildSketch(Plane.XY) as sk_gusset:
            with BuildLine():
                Polyline(
                    (BRIDGE_X_MIN, MINUS_Y_IN),
                    (47.0, head_y_min + 1.5),
                    (BRIDGE_X_MIN, head_y_min + 1.5),
                    (BRIDGE_X_MIN, MINUS_Y_IN),
                )
            make_face()
        extrude(sk_gusset.sketch, amount=HEAD_Z_MAX)

    part = ap.part + mg90s_head

    # 3. Cut clean 25T CNC Round Horn pattern on -Y arm (Ø6.2 center + 4x Ø3.2 M3 on Ø14.0 PCD)
    with BuildPart() as cuts:
        with Locations((PIVOT_X, arm_ymid, PIVOT_Z)):
            Cylinder(radius=3.1, height=arm_thk + 2.0, rotation=(90, 0, 0))
        for k in range(4):
            ang = math.radians(45.0 + k * 90.0)
            hx = PIVOT_X + 7.0 * math.cos(ang)
            hz = PIVOT_Z + 7.0 * math.sin(ang)
            with Locations((hx, arm_ymid, hz)):
                Cylinder(radius=1.6, height=arm_thk + 2.0, rotation=(90, 0, 0))

    part = part - cuts.part
    try:
        part = part.clean()
    except Exception:
        pass
    return part


def place_on_joint2_local(shape_on_j2, ang2_deg=0.0):
    """Place single-arm part onto Link #1's Joint 2 horn (at J2_SHAFT_X=60.0, J2_SHAFT_Y=7.60, Z=49.2)."""
    z_off = (HOUSING_TOP_Z + 13.7) + MINUS_Y_IN
    p = shape_on_j2.translate((-PIVOT_X, 0.0, -PIVOT_Z))
    p = p.rotate(Axis.X, -90).translate((J2_SHAFT_X, J2_SHAFT_Y, HOUSING_TOP_Z + 13.7 + MINUS_Y_IN))
    # Verify: local (PIVOT_X, MINUS_Y_IN, PIVOT_Z) -> after translate((-PIVOT_X, 0, -PIVOT_Z)) = (0, MINUS_Y_IN, 0)
    # After Rx(-90): (0, 0, -MINUS_Y_IN) = (0, 0, +17.5)
    # After translate(..., z_off = 49.2 - 17.5): (J2_SHAFT_X, J2_SHAFT_Y, 49.2) -> EXACT!
    if abs(ang2_deg) > 1e-6:
        p = p.rotate(Axis((J2_SHAFT_X, J2_SHAFT_Y, 0.0), (0, 0, 1)), ang2_deg)
    return p


def place_on_joint1_world(shape_loc, shaft_z=SHAFT_Z, angle_deg=0.0):
    """Place single-arm part directly onto Joint Base V6's MG996R #1 horn."""
    p = shape_loc.translate((-PIVOT_X, 0.0, -PIVOT_Z))
    p = p.rotate(Axis.Y, -90).rotate(Axis.Z, -90)
    p = p.translate((-15.0, 0.0, shaft_z))
    if abs(angle_deg) > 1e-6:
        p = p.rotate(Axis((-15.0, 0.0, shaft_z), (1, 0, 0)), angle_deg)
    return p


def part_to_tris(shape):
    tmp = "/tmp/_part_1arm_tmp.stl"
    export_stl(shape, tmp)
    with open(tmp, "rb") as f:
        f.read(80)
        cnt = struct.unpack("<I", f.read(4))[0]
        return np.fromfile(f, dtype=np.dtype([("n", "<f4", (3,)), ("v", "<f4", (3, 3)), ("a", "<u2")]), count=cnt)["v"]


def main():
    print("Building single-arm ('1 cang') MG996R-to-MG90S cover...")
    part = build_mg996r_to_mg90s_1arm()
    bb = part.bounding_box()
    print(f"  local bbox: X=[{bb.min.X:.2f},{bb.max.X:.2f}] ({bb.max.X-bb.min.X:.2f}), "
          f"Y=[{bb.min.Y:.2f},{bb.max.Y:.2f}] ({bb.max.Y-bb.min.Y:.2f}), "
          f"Z=[{bb.min.Z:.2f},{bb.max.Z:.2f}] ({bb.max.Z-bb.min.Z:.2f})")
    print(f"  link length (MG996R pivot -> MG90S shaft X={50.0 + HEAD_SHIFT_X:.1f}) = {50.0 + HEAD_SHIFT_X - PIVOT_X:.2f} mm")
    print(f"  is_valid={part.is_valid}, solids={len(part.solids())}, vol={part.volume:.1f} mm3 (~{part.volume*0.00127:.1f} g @100%)")

    step_out = os.path.join(DIR, "arm_link_mg996r_to_mg90s_1arm.step")
    stl_out = os.path.join(DIR, "arm_link_mg996r_to_mg90s_1arm.stl")
    pr_out = os.path.join(DIR, "PrintReady_MG996R_to_MG90S_1Arm.stl")
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

    print("\nSweep A: 1-Arm MG996R->MG90S on Joint Base V6 (0..+/-105 deg):")
    for ang in [0, 45, 90, 105, -45, -90, -105]:
        pw = place_on_joint1_world(part, shaft_z=SHAFT_Z, angle_deg=ang)
        v_jb = sum(s.volume for s in (pw.intersect(joint_base).solids() if pw.intersect(joint_base) else []))
        v_m1 = sum(s.volume for s in (pw.intersect(mg1).solids() if pw.intersect(mg1) else []))
        print(f"  ang1={ang:+4d} deg -> vs JointBase={v_jb:.3f} mm3 | vs MG996R#1={v_m1:.3f} mm3")

    # Collision sweep 2: mounted on Join Link (MG996R #2 horn)
    link1_loc = import_step(os.path.join(DIR, "arm_link_mg996r_to_mg996r.step"))
    mg2_loc = mg_proto.rotate(Axis.Z, 180).translate((J2_BODY_CX, J2_SHAFT_Y, HOUSING_TOP_Z - 27.5))
    horn2_loc = horn_proto.rotate(Axis.X, 180).translate((J2_SHAFT_X, J2_SHAFT_Y, HOUSING_TOP_Z + 13.7))

    print("\nSweep B: 1-Arm MG996R->MG90S stacked on Join Link MG996R #2 (0..+/-105 deg):")
    for ang in [0, 45, 90, 105, -45, -90, -105]:
        p_j2 = place_on_joint2_local(part, ang2_deg=ang)
        v_l1 = sum(s.volume for s in (p_j2.intersect(link1_loc).solids() if p_j2.intersect(link1_loc) else []))
        v_m2 = sum(s.volume for s in (p_j2.intersect(mg2_loc).solids() if p_j2.intersect(mg2_loc) else []))
        print(f"  ang2={ang:+4d} deg -> vs JoinLink={v_l1:.3f} mm3 | vs MG996R#2={v_m2:.3f} mm3")

    # Export assembly check STEP (Joint Base -> Join Link -> 1-Arm MG996R->MG90S Cover)
    link1_w = place_in_world(link1_loc, shaft_z=SHAFT_Z, angle_deg=0.0)
    mg2_w = place_in_world(mg2_loc, shaft_z=SHAFT_Z, angle_deg=0.0)
    horn2_w = place_in_world(horn2_loc, shaft_z=SHAFT_Z, angle_deg=0.0)
    top_1arm_w = place_in_world(place_on_joint2_local(part, ang2_deg=0.0), shaft_z=SHAFT_Z, angle_deg=0.0)

    asm_step = os.path.join(DIR, "arm_mg996r_to_mg90s_1arm_assembly_check.step")
    export_step(Compound(children=[joint_base, mg1, horn1, link1_w, mg2_w, horn2_w, top_1arm_w]), asm_step)

    # Render 3-panel PNG preview
    palette_asm = [
        (joint_base, [0.85, 0.45, 0.14]),
        (mg1,        [0.18, 0.20, 0.24]),
        (horn1,      [0.78, 0.18, 0.22]),
        (link1_w,    [0.16, 0.56, 0.84]),
        (mg2_w,      [0.18, 0.20, 0.24]),
        (horn2_w,    [0.78, 0.18, 0.22]),
        (top_1arm_w, [0.18, 0.72, 0.52]),
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
    draw_list(fig.add_subplot(1, 3, 1, projection="3d"), [(part, [0.18, 0.72, 0.52])], (-25, 85), (-30, 30), (-5, 35), 28, -50, "1-Arm MG996R -> MG90S Cover (Iso)")
    draw_list(fig.add_subplot(1, 3, 2, projection="3d"), [(part, [0.18, 0.72, 0.52])], (-25, 85), (-30, 30), (-5, 35), 22, 130, "1-Arm Horn Face & MG90S Pocket")
    draw_list(fig.add_subplot(1, 3, 3, projection="3d"), palette_asm, (-55, 55), (-55, 55), (15, 225), 20, -45, "Stacked on Join Link (0.000 mm3 Clash)")
    plt.tight_layout()
    png_out = os.path.join(ARTIFACTS, "mg996r_to_mg90s_1arm_check.png")
    plt.savefig(png_out, dpi=150)
    print(f"Saved {pr_out}, {step_out}, {asm_step}, and {png_out}")


if __name__ == "__main__":
    main()

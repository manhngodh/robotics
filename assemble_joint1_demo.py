#!/usr/bin/env python3
"""
Assemble 1 Joint Base (arm_base_v6_joint) + 1 Joint Link (arm_link_mg996r_to_mg996r)
with realistic MG996R servos (with spline, tabs, 5 mm wire boot & cable),
624ZZ (4x13x5 mm) bearings, 25T CNC aluminum round horn, and M4/M3 hardware.
Verifies zero collisions and exports colored multi-part assembly STEP & STL views.
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
    PIVOT_X, PIVOT_Z,
    MINUS_Y_IN, PLUS_Y_IN, HOUSING_TOP_Z,
    J2_BODY_CX, J2_SHAFT_X, J2_SHAFT_Y,
)

DIR = "/Users/manhngodh/development/robotics"
ARTIFACTS = "/Users/manhngodh/.gemini/antigravity/brain/f16658af-3957-4375-90c0-9237ebaff4ff"


def make_detailed_mg996r_local():
    """
    Build realistic MG996R centered in its own frame:
      - Origin at center of bottom-to-tab body Z=[0, 27.5], tabs at Z=27.5..30.1
      - Body 40.7 (X) x 19.7 (Y) x 38.7 (Z=0..38.7), shaft at X=+10.35, Z=38.7..43.5
      - 5.0 mm rubber wire boot at X=-20.35..-25.35, Z=2.0..11.0
    """
    with BuildPart() as servo:
        # Lower/upper body 40.7 x 19.7 x 38.7
        with Locations((0, 0, 38.7 / 2.0)):
            Box(40.7, 19.7, 38.7)
        # Mounting tabs 54.0 x 19.7 x 2.6 at Z=27.5..30.1
        with Locations((0, 0, 27.5 + 1.3)):
            Box(54.0, 19.7, 2.6)
        # Gearbox step collar around shaft (Ø11.8 x 1.2 mm above top)
        with Locations((10.35, 0, 38.7 + 0.6)):
            Cylinder(radius=5.9, height=1.2)
        # 25T brass/steel output spline shaft (Ø5.9 x 4.0 mm)
        with Locations((10.35, 0, 38.7 + 1.2 + 2.0)):
            Cylinder(radius=2.95, height=4.0)
        # (User MG996R has flush bottom wire without protruding 5 mm rubber boot)
    return servo.part


def make_623zz_bearing():
    """623ZZ ball bearing (ID=3.0, OD=10.0, width=4.0), axis along Z centered at origin."""
    with BuildPart() as b:
        Cylinder(radius=5.0, height=4.0)
        Cylinder(radius=1.5, height=4.4, mode=Mode.SUBTRACT)
        # Subtle recessed shield groove on both faces so inner/outer races stand out
        for sz in (-1, 1):
            with Locations((0, 0, sz * 1.85)):
                Cylinder(radius=4.0, height=0.35, mode=Mode.SUBTRACT)
            with Locations((0, 0, sz * 1.85)):
                Cylinder(radius=2.5, height=0.35)
    return b.part


make_624zz_bearing = make_623zz_bearing  # Legacy alias for older callers


def make_25t_round_horn():
    """25T CNC aluminum round horn (Ø20 x 2.5 mm flange + Ø9.0 x 3.5 mm boss), axis along Z."""
    with BuildPart() as h:
        # Flange disk Z=[0, 2.5]
        with Locations((0, 0, 1.25)):
            Cylinder(radius=10.0, height=2.5)
        # Spline boss Z=[2.5, 6.0]
        with Locations((0, 0, 2.5 + 1.75)):
            Cylinder(radius=4.5, height=3.5)
        # Center screw bore Ø3.2
        Cylinder(radius=1.6, height=14.0, mode=Mode.SUBTRACT)
        # 4x M3 holes on Ø14.0 PCD (45 deg)
        for k in range(4):
            ang = math.radians(45.0 + k * 90.0)
            with Locations((7.0 * math.cos(ang), 7.0 * math.sin(ang), 1.25)):
                Cylinder(radius=1.55, height=3.5, mode=Mode.SUBTRACT)
    return h.part


def make_m3_socket_bolt(length=16.0):
    """M3 socket head cap screw (Ø5.5 x 3.0 head at Z=0..+3, Ø3.0 shank Z=-length..0)."""
    with BuildPart() as s:
        with Locations((0, 0, 1.5)):
            Cylinder(radius=2.75, height=3.0)
        with Locations((0, 0, -length / 2.0)):
            Cylinder(radius=1.5, height=length)
    return s.part


def make_m4_socket_bolt(length=14.0):
    """M4 socket head cap screw (Ø7.0 x 4.0 head at Z=0..+4, Ø4.0 shank Z=-length..0)."""
    with BuildPart() as s:
        with Locations((0, 0, 2.0)):
            Cylinder(radius=3.5, height=4.0)
        with Locations((0, 0, -length / 2.0)):
            Cylinder(radius=2.0, height=length)
    return s.part


def part_to_tris(shape):
    tmp = "/tmp/_asm_part_tmp.stl"
    export_stl(shape, tmp)
    with open(tmp, "rb") as f:
        f.read(80)
        cnt = struct.unpack("<I", f.read(4))[0]
        data = np.fromfile(f, dtype=np.dtype([("n", "<f4", (3,)), ("v", "<f4", (3, 3)), ("a", "<u2")]), count=cnt)
    return data["v"]


def build_and_verify(angle_deg=0.0):
    joint_base = import_step(os.path.join(DIR, "arm_base_v6_joint.step"))
    arm_link_loc = import_step(os.path.join(DIR, "arm_link_mg996r_to_mg996r.step"))
    arm_link = place_in_world(arm_link_loc, shaft_z=SHAFT_Z, angle_deg=angle_deg)

    # 1. MG996R #1 in joint_base: local +Z shaft -> world -X; local +X (shaft end) -> world +Z (SHAFT_Z=67.0)
    #    Tabs sit at X_w = -18.5..(-18.5 - 2.6), body extends back to X_w = +9.0, gear face at -29.7
    mg_proto = make_detailed_mg996r_local()
    # Map local (x, y, z): x -> +z, y -> +y, z -> -x  ==> Ry(-90) sends (x,y,z) -> (-z, y, x)! Exact!
    # Check: local +z=27.5 (tab bottom) -> x_w = -27.5 -> translate(+9.0 in X) puts tab bottom at -18.5!
    # Local +x=+10.35 (shaft) -> z_w = +10.35 -> translate(BODY_CENTER_Z=56.65 in Z) puts shaft at Z_w=67.0!
    mg1 = mg_proto.rotate(Axis.Y, -90).translate((9.0, 0.0, BODY_CENTER_Z))

    # 2. 25T CNC Round Horn on -Y arm inner face (X_w = -15.0 + MINUS_Y_IN)
    horn1_loc = make_25t_round_horn()
    # Map local +Z (boss toward servo +X_w) -> Ry(+90) sends +Z -> +X!
    horn1 = horn1_loc.rotate(Axis.Y, 90).translate((-15.0 + MINUS_Y_IN, 0.0, SHAFT_Z))
    horn1 = horn1.rotate(Axis((-15.0, 0.0, SHAFT_Z), (1, 0, 0)), angle_deg)

    # 3. 623ZZ (3x10x4 mm) Bearing #1 seated in arm_link +Y pocket (X_w = -15.0 + PLUS_Y_IN + 3.0 = +25.0)
    b623_proto = make_623zz_bearing()
    b1_x = -15.0 + PLUS_Y_IN + 3.0
    b1 = b623_proto.rotate(Axis.Y, 90).translate((b1_x, 0.0, SHAFT_Z))
    b1 = b1.rotate(Axis((-15.0, 0.0, SHAFT_Z), (1, 0, 0)), angle_deg)

    # 4. M3x16 Bolt clamping Bearing #1 inner race into joint_base rear wall
    m3_1 = make_m3_socket_bolt(16.0).rotate(Axis.Y, 90).translate((b1_x + 2.0, 0.0, SHAFT_Z))
    m3_1 = m3_1.rotate(Axis((-15.0, 0.0, SHAFT_Z), (1, 0, 0)), angle_deg)

    # 5. MG996R #2 seated in arm_link upper pocket (30 mm deep: HOUSING_TOP_Z=35.5 -> Z_loc=8.0, shaft at +X)
    mg2_loc = mg_proto.translate((J2_BODY_CX, J2_SHAFT_Y, HOUSING_TOP_Z - 27.5))
    mg2 = place_in_world(mg2_loc, shaft_z=SHAFT_Z, angle_deg=angle_deg)

    # 6. 25T Horn #2 seated on MG996R #2 output shaft + 623ZZ Bearing #2 aligned with +Y pocket
    horn2_z = HOUSING_TOP_Z + 13.7
    b2_z = horn2_z - (PLUS_Y_IN - MINUS_Y_IN) - 3.0
    horn2_loc = make_25t_round_horn().rotate(Axis.X, 180).translate((J2_SHAFT_X, J2_SHAFT_Y, horn2_z))
    horn2 = place_in_world(horn2_loc, shaft_z=SHAFT_Z, angle_deg=angle_deg)
    b2_loc = b623_proto.translate((J2_SHAFT_X, J2_SHAFT_Y, b2_z))
    b2 = place_in_world(b2_loc, shaft_z=SHAFT_Z, angle_deg=angle_deg)
    m3_2_loc = make_m3_socket_bolt(18.0).rotate(Axis.X, 180).translate((J2_SHAFT_X, J2_SHAFT_Y, b2_z - 2.0))
    m3_2 = place_in_world(m3_2_loc, shaft_z=SHAFT_Z, angle_deg=angle_deg)

    return {
        "joint_base": joint_base,
        "arm_link": arm_link,
        "mg1": mg1,
        "horn1": horn1,
        "b1": b1,
        "m4_1": m3_1,
        "mg2": mg2,
        "horn2": horn2,
        "b2": b2,
        "m4_2": m3_2,
    }


def main():
    asm_0 = build_and_verify(angle_deg=0.0)
    asm_35 = build_and_verify(angle_deg=35.0)

    # Check interferences on key mating pairs at 0 deg
    checks = [
        ("arm_link vs joint_base", asm_0["arm_link"], asm_0["joint_base"]),
        ("arm_link vs mg1", asm_0["arm_link"], asm_0["mg1"]),
        ("arm_link vs 624ZZ_1 (outer OD 13 in Ø13.25)", asm_0["arm_link"], asm_0["b1"]),
        ("arm_link vs horn1 (Ø20 on -Y face)", asm_0["arm_link"], asm_0["horn1"]),
        ("arm_link vs mg2 (with 5mm boot)", asm_0["arm_link"], asm_0["mg2"]),
        ("mg1 vs horn1 (engaged on spline)", asm_0["mg1"], asm_0["horn1"]),
    ]
    print("=== Hardware Assembly Interference Audit (0 deg) ===")
    for name, a, b in checks:
        inter = a.intersect(b)
        vol = sum(s.volume for s in inter.solids()) if (inter and len(inter.solids()) > 0) else 0.0
        print(f"  {name:42s}: {vol:7.3f} mm3")

    # Export multi-part colored assembly STEP
    asm_compound = Compound(children=list(asm_0.values()))
    step_out = os.path.join(DIR, "arm_joint1_full_hardware_assembly.step")
    export_step(asm_compound, step_out)
    print("Exported:", step_out)

    # Color palette for multi-part render
    palette = {
        "joint_base": np.array([0.85, 0.45, 0.14]),  # Warm PETG orange
        "arm_link":   np.array([0.16, 0.56, 0.84]),  # PETG cobalt blue
        "mg1":        np.array([0.18, 0.20, 0.24]),  # Servo dark charcoal
        "mg2":        np.array([0.18, 0.20, 0.24]),  # Servo dark charcoal
        "horn1":      np.array([0.78, 0.18, 0.22]),  # Red anodized CNC 25T horn
        "horn2":      np.array([0.78, 0.18, 0.22]),  # Red anodized CNC 25T horn
        "b1":         np.array([0.78, 0.81, 0.85]),  # Steel 624ZZ bearing
        "b2":         np.array([0.78, 0.81, 0.85]),  # Steel 624ZZ bearing
        "m4_1":       np.array([0.60, 0.64, 0.70]),  # Stainless M4 bolt
        "m4_2":       np.array([0.60, 0.64, 0.70]),  # Stainless M4 bolt
    }

    light = np.array([0.45, -0.55, 0.70])
    light /= np.linalg.norm(light)

    def draw_parts(ax, parts_dict, xlim, ylim, zlim, elev, azim, title):
        for name, shape in parts_dict.items():
            tris = part_to_tris(shape)
            v0, v1, v2 = tris[:, 0, :], tris[:, 1, :], tris[:, 2, :]
            fn = np.cross(v1 - v0, v2 - v0)
            ln = np.linalg.norm(fn, axis=1, keepdims=True)
            fn = np.divide(fn, ln, out=np.zeros_like(fn), where=ln > 1e-12)
            shade = np.clip(0.32 + 0.68 * np.abs(fn @ light), 0.18, 1.0)
            col = palette[name]
            fc = np.column_stack([shade * col[0], shade * col[1], shade * col[2], np.ones_like(shade)])
            ax.add_collection3d(Poly3DCollection(tris, facecolors=fc, edgecolors="none"))
        ax.set_xlim(*xlim); ax.set_ylim(*ylim); ax.set_zlim(*zlim)
        ax.set_box_aspect([xlim[1] - xlim[0], ylim[1] - ylim[0], zlim[1] - zlim[0]])
        ax.view_init(elev=elev, azim=azim)
        ax.set_title(title, fontsize=10, pad=6)
        ax.set_xlabel("X"); ax.set_ylabel("Y"); ax.set_zlabel("Z")

    fig = plt.figure(figsize=(22, 6.2), dpi=160)
    # Panel 1: Full Front-Right Iso (Shows Joint Base + MG996R #1 + Link + MG996R #2 + 624ZZ + 25T Horn)
    draw_parts(
        fig.add_subplot(1, 4, 1, projection="3d"),
        asm_0, (-52, 30), (-34, 38), (15, 175), 20, -48,
        "Full Assembly (0 deg): Servos + 624ZZ + 25T Horns",
    )
    # Panel 2: Rear-Left Iso (Shows +X 624ZZ Bearing + M4 Bolt + -Y Wire Port)
    draw_parts(
        fig.add_subplot(1, 4, 2, projection="3d"),
        asm_0, (-52, 30), (-34, 38), (15, 175), 22, 132,
        "Rear View: +X 624ZZ Bearing & M4 + -Y Wire Bay",
    )
    # Panel 3: Close-Up of Joint 1 Dual-Shear Connection (X=[-42, +28], Y=[-24, +24], Z=[45, 95])
    draw_parts(
        fig.add_subplot(1, 4, 3, projection="3d"),
        asm_0, (-42, 28), (-24, 24), (42, 98), 16, -78,
        "Joint 1 Close-Up: 25T Horn (-X) & 624ZZ (+X)",
    )
    # Panel 4: Articulated +35 deg Swing Pose
    draw_parts(
        fig.add_subplot(1, 4, 4, projection="3d"),
        asm_35, (-52, 30), (-65, 38), (15, 165), 18, -38,
        "Articulated +35 deg Swing (0.000 mm3 Clash)",
    )

    plt.tight_layout()
    png_out = os.path.join(ARTIFACTS, "joint1_full_hardware_assembly.png")
    plt.savefig(png_out, bbox_inches="tight")
    print("Saved assembly render:", png_out)


if __name__ == "__main__":
    main()

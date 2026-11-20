#!/usr/bin/env python3
"""
Assemble 1 Joint Base (arm_base_v6_joint) + Join #1 (arm_link_mg996r_to_mg996r)
+ Join #2 (2nd arm_link_mg996r_to_mg996r stacked onto Joint 2 at +90 deg),
complete with 3x MG996R servos, 3x 624ZZ (4x13x5) bearings, 3x 25T CNC horns, and M4 bolts.
"""

import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from build123d import *

from build_arm_base_v6 import SHAFT_Z
from build_mg996r_to_mg996r import (
    place_in_world,
    PIVOT_X, PIVOT_Z,
    MINUS_Y_IN, PLUS_Y_IN, HOUSING_TOP_Z,
    J2_BODY_CX, J2_SHAFT_X, J2_SHAFT_Y,
)
from assemble_joint1_demo import (
    build_and_verify,
    make_detailed_mg996r_local,
    make_623zz_bearing,
    make_25t_round_horn,
    make_m3_socket_bolt,
    part_to_tris,
)

DIR = "/Users/manhngodh/development/robotics"
ARTIFACTS = "/Users/manhngodh/.gemini/antigravity/brain/f16658af-3957-4375-90c0-9237ebaff4ff"


def place_on_joint2_local(shape_on_j2, ang2_deg=90.0):
    """Map Link #2's local Joint 1 frame onto Link #1's local Joint 2 axis (+Z at (J2_SHAFT_X, J2_SHAFT_Y))."""
    z_off = (HOUSING_TOP_Z + 13.7) + MINUS_Y_IN
    p = shape_on_j2.rotate(Axis.X, -90).translate((J2_SHAFT_X - PIVOT_X, J2_SHAFT_Y - PIVOT_Z, z_off))
    p = p.rotate(Axis((J2_SHAFT_X, J2_SHAFT_Y, 0), (0, 0, 1)), ang2_deg)
    return p


def main():
    ang1 = 0.0
    ang2 = 0.0  # Straight upright 0 deg! (Now J2 shaft is at +X outer tip R<=19.0 mm -> 0..+/-105 deg!)

    parts = build_and_verify(angle_deg=ang1)
    link_proto = import_step(os.path.join(DIR, "arm_link_mg996r_to_mg996r.step"))
    mg_proto = make_detailed_mg996r_local()
    b623_proto = make_623zz_bearing()
    horn_proto = make_25t_round_horn()

    # Sweep Join #2 across Joint 2 angles [0, +/-45, +/-90, +/-105 deg] to verify full range!
    print(f"=== Join #2 Swing Sweep on Join #1 (J2_SHAFT_X={J2_SHAFT_X}) ===")
    for sw in [0, 45, 90, 105, -45, -90, -105]:
        l2_test = place_in_world(place_on_joint2_local(link_proto, ang2_deg=sw), shaft_z=SHAFT_Z, angle_deg=ang1)
        i1 = l2_test.intersect(parts["arm_link"])
        v1 = sum(s.volume for s in i1.solids()) if (i1 and len(i1.solids()) > 0) else 0.0
        i2 = l2_test.intersect(parts["mg2"])
        v2 = sum(s.volume for s in i2.solids()) if (i2 and len(i2.solids()) > 0) else 0.0
        print(f"  ang2={sw:+4d} deg -> Link2 vs Link1={v1:.3f} mm3 | Link2 vs MG996R#2={v2:.3f} mm3")

    # Link #2 stacked onto Joint 2 at ang2 = 0 deg (straight)
    link2_loc = place_on_joint2_local(link_proto, ang2_deg=ang2)
    link2_w = place_in_world(link2_loc, shaft_z=SHAFT_Z, angle_deg=ang1)

    # MG996R #3 inside Link #2's upper pocket (30 mm deep: HOUSING_TOP_Z=35.5 -> Z_loc=8.0, shaft at +X)
    mg3_on_l2 = mg_proto.translate((J2_BODY_CX, J2_SHAFT_Y, HOUSING_TOP_Z - 27.5))
    mg3_w = place_in_world(place_on_joint2_local(mg3_on_l2, ang2_deg=ang2), shaft_z=SHAFT_Z, angle_deg=ang1)

    # 25T Horn #3 + 623ZZ Bearing #3 + M3 Bolt #3 on Joint 3
    horn3_z = HOUSING_TOP_Z + 13.7
    b3_z = horn3_z - (PLUS_Y_IN - MINUS_Y_IN) - 3.0
    horn3_on_l2 = horn_proto.rotate(Axis.X, 180).translate((J2_SHAFT_X, J2_SHAFT_Y, horn3_z))
    horn3_w = place_in_world(place_on_joint2_local(horn3_on_l2, ang2_deg=ang2), shaft_z=SHAFT_Z, angle_deg=ang1)
    b3_on_l2 = b623_proto.translate((J2_SHAFT_X, J2_SHAFT_Y, b3_z))
    b3_w = place_in_world(place_on_joint2_local(b3_on_l2, ang2_deg=ang2), shaft_z=SHAFT_Z, angle_deg=ang1)
    m3_3_on_l2 = make_m3_socket_bolt(18.0).rotate(Axis.X, 180).translate((J2_SHAFT_X, J2_SHAFT_Y, b3_z - 2.0))
    m3_3_w = place_in_world(place_on_joint2_local(m3_3_on_l2, ang2_deg=ang2), shaft_z=SHAFT_Z, angle_deg=ang1)

    parts_2j = dict(parts)
    parts_2j.update({
        "arm_link2": link2_w,
        "mg3": mg3_w,
        "horn3": horn3_w,
        "b3": b3_w,
        "m4_3": m3_3_w,
    })

    # Interference summary
    for label, a, b in [
        ("Link2 vs Link1 (ang2=+90)", link2_w, parts["arm_link"]),
        ("Link2 vs JointBase", link2_w, parts["joint_base"]),
        ("Link2 vs MG996R #2 (collar touch)", link2_w, parts["mg2"]),
        ("Link2 vs 624ZZ #2", link2_w, parts["b2"]),
        ("MG996R #3 vs Link2", mg3_w, link2_w),
    ]:
        inter = a.intersect(b)
        v = sum(s.volume for s in inter.solids()) if (inter and len(inter.solids()) > 0) else 0.0
        print(f"  {label:35s}: {v:7.3f} mm3")

    # Export STEP + STL
    comp = Compound(children=list(parts_2j.values()))
    step_out = os.path.join(DIR, "arm_2joints_full_hardware_assembly.step")
    stl_out = os.path.join(DIR, "arm_2joints_full_hardware_assembly.stl")
    export_step(comp, step_out)
    export_stl(comp, stl_out)
    print("Exported:", step_out, "and", stl_out)

    # Color render (3 angles)
    palette = {
        "joint_base": np.array([0.85, 0.45, 0.14]),  # Orange base
        "arm_link":   np.array([0.16, 0.56, 0.84]),  # Blue Join #1
        "arm_link2":  np.array([0.18, 0.72, 0.52]),  # Teal-Green Join #2
        "mg1":        np.array([0.18, 0.20, 0.24]),
        "mg2":        np.array([0.18, 0.20, 0.24]),
        "mg3":        np.array([0.18, 0.20, 0.24]),
        "horn1":      np.array([0.80, 0.18, 0.22]),
        "horn2":      np.array([0.80, 0.18, 0.22]),
        "horn3":      np.array([0.80, 0.18, 0.22]),
        "b1":         np.array([0.78, 0.81, 0.86]),
        "b2":         np.array([0.78, 0.81, 0.86]),
        "b3":         np.array([0.78, 0.81, 0.86]),
        "m4_1":       np.array([0.55, 0.60, 0.66]),
        "m4_2":       np.array([0.55, 0.60, 0.66]),
        "m4_3":       np.array([0.55, 0.60, 0.66]),
    }
    light = np.array([0.45, -0.55, 0.70]); light /= np.linalg.norm(light)

    def draw(ax, elev, azim, xlim, ylim, zlim, title):
        for k, sh in parts_2j.items():
            tris = part_to_tris(sh)
            v0, v1, v2 = tris[:, 0, :], tris[:, 1, :], tris[:, 2, :]
            fn = np.cross(v1 - v0, v2 - v0)
            ln = np.linalg.norm(fn, axis=1, keepdims=True)
            fn = np.divide(fn, ln, out=np.zeros_like(fn), where=ln > 1e-12)
            shade = np.clip(0.32 + 0.68 * np.abs(fn @ light), 0.18, 1.0)
            col = palette[k]
            fc = np.column_stack([shade * col[0], shade * col[1], shade * col[2], np.ones_like(shade)])
            ax.add_collection3d(Poly3DCollection(tris, facecolors=fc, edgecolors="none"))
        ax.set_xlim(*xlim); ax.set_ylim(*ylim); ax.set_zlim(*zlim)
        ax.set_box_aspect([xlim[1] - xlim[0], ylim[1] - ylim[0], zlim[1] - zlim[0]])
        ax.view_init(elev=elev, azim=azim)
        ax.set_title(title, fontsize=10, pad=6)
        ax.set_xlabel("X"); ax.set_ylabel("Y"); ax.set_zlabel("Z")

    fig = plt.figure(figsize=(19, 6.2), dpi=160)
    draw(fig.add_subplot(1, 3, 1, projection="3d"), 20, -45, (-55, 45), (-35, 35), (15, 255), "2-Joint Upright Assembly (0 deg + 0 deg)")
    draw(fig.add_subplot(1, 3, 2, projection="3d"), 22, 135, (-55, 45), (-35, 35), (15, 255), "Rear-Iso (All 3 MG996R, 624ZZ & 25T Horns)")
    draw(fig.add_subplot(1, 3, 3, projection="3d"), 14, -78, (-55, 45), (-25, 25), (110, 195), "Joint 2 Connection Close-Up (Straight 0 deg)")
    plt.tight_layout()
    png_out = os.path.join(ARTIFACTS, "joint2_full_hardware_assembly.png")
    plt.savefig(png_out, bbox_inches="tight")
    print("Saved render:", png_out)


if __name__ == "__main__":
    main()

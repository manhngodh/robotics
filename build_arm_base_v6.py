#!/usr/bin/env python3
"""
Modular C-Clamp Base + Dedicated Monolithic Standing Servo Joint Pylon (V6)
for M Robot / MD Station.

Key improvement over V5:
Instead of clamping onto Cover<1>'s flimsy open U-clevis with a 1-screw pin,
arm_base_v6_joint REPLACES the bottom u_break_cover shell entirely as ONE solid,
heavily ribbed bolted pylon:
  - 10 mm thick bolted base flange (4x M4 bolts + 3 mm shear key) on clamp deck
  - Wide 4-ribbed flared pedestal (Z=29..56 mm) tapering smoothly to the upper servo neck
  - Integrated MG90S #1 housing pocket + cable channel + +X M3/8mm-bearing boss
    holding Joint 1 directly at (-15.0, 0.0, 82.0) mm!
"""

import os
import math
from build123d import *

OUT_DIR = "/Users/manhngodh/development/robotics"

# ====================================================================
# 1. COORDINATES
#    Desk top: Z = 0.0 mm
#    C-clamp deck top: Z = 19.0 mm (X=-48..+18, Y=+/-30.25)
#    Joint 1 servo shaft center: X = -15.0, Y = 0.0, Z = 82.0 mm
#      (virtual Cover<1> frame: R_stand = Ry(-90), t_cov0 = [+0.5, 0.0, 33.25])
#      -> MG90S #1 at [-2.5, 0.0, 77.75] (body Z_loc=3..25.5 -> X_w=-2.5..-25.0)
#      -> Cover<2> -Y arm (Ø4.8/M2.5) at X_w = -32.5..-37.0 (faces MG90S spline!)
#      -> Cover<2> +Y arm (Ø8.2/M3) at X_w = +2.5..+7.0
# ====================================================================
BACK_X_OUTER = -48.0
BACK_X_INNER = -32.0
JAW_X_FRONT = 18.0
DECK_Z_TOP = 19.0
FRAME_HALF_W = 30.25

THROAT_OPENING = 66.0       # Supports 10-50 mm desk thickness (compact & stiff for 13 mm desk!)
LOWER_JAW_T = 18.0
LOWER_JAW_Z_TOP = -THROAT_OPENING
LOWER_JAW_Z_BOT = LOWER_JAW_Z_TOP - LOWER_JAW_T  # -84.0 mm

# Shear Keyway + 4x M4 Pattern on Z=19.0 Deck
KEY_X_MIN = -36.0
KEY_X_MAX = +6.0
KEY_HALF_W = 16.0
KEY_DEPTH = 3.0

M4_BOLT_XS = [-41.0, 11.0]
M4_BOLT_Y = 23.0
M4_HOLE_R = 2.2
M4_CBORE_R = 3.9
M4_NUT_AF = 7.2
M4_NUT_H = 3.5

# Monolithic Base-to-Servo Joint Pylon (arm_base_v6_joint) — MG996R Edition
JOINT_X_MIN = -47.0
JOINT_X_MAX = 17.0
JOINT_HALF_W = 29.0
FLANGE_T = 8.0              # Compact 8 mm bolted base plate (Z=19..27)
FLANGE_Z_TOP = DECK_Z_TOP + FLANGE_T  # 27.0 mm

# Joint 1 MG996R servo head coords (dual-shear symmetric axis at Y=0, Z=SHAFT_Z=67.0)
# Direct base + servo cover layout: eliminates the tall wasted pedestal block!
SHAFT_X = -15.0
SHAFT_Y = 0.0
NECK_START_Z = FLANGE_Z_TOP # 27.0 mm: servo housing sits directly on base flange
SHAFT_Z = NECK_START_Z + 40.0  # 67.0 mm (leaves 3.95 mm under lower M3 bore & >=24.5 mm Cover<2> clearance at +/-90 deg)
BODY_CENTER_Z = SHAFT_Z - 10.35   # 56.65 mm (MG996R shaft is 10.0 mm from short upper end -> 10.35 mm above center)
HEAD_HALF_Y = 15.5          # 31.0 mm outer head width (5.3 mm reinforced side walls around 20.4 mm pocket)
HEAD_TOP_Y = 9.0            # Reinforced beveled roof: corner R=sqrt(9^2+18^2)=20.12 mm (<21.0 mm bridge clearance)
HEAD_TOP_Z = SHAFT_Z + 18.0 # 85.0 mm (2.65 mm solid PETG above top M3 holes at Z=81.15)
HEAD_X_FRONT = -18.5        # Mounting tab seat plane (2.8 mm gap from MG996R gear face -29.7 to -Y arm -32.5 for 25T horn)
POCKET_X_BACK = +11.5       # Closed inner back wall (30.00 mm pocket depth from HEAD_X_FRONT=-18.5!)
HEAD_X_BACK = +16.5         # 5.0 mm reinforced solid rear bearing bridge wall (+11.5..+16.5, inside JOINT_X_MAX=+17.0)
BEARING_BOSS_LEN = 1.0      # +X inner-race spigot (+16.5..+17.5) -> 0.2 mm clear of widened +Y arm (+17.7..+23.7)

# Wing span cleanly inside M4 bolt keep-out zones (bolts at X=-41, +11 with R=4.0 counterbore)
WING_X_MIN = -33.5          # 3.5 mm clear of rear M4 counterbores (edge X=-37.0)
WING_X_MAX = +3.5           # 3.5 mm clear of front M4 counterbores (edge X=+7.0)

# MG996R pocket inside upper pylon head (40.7 x 19.7 mm body + 0.6/0.7 mm print clearance)
POCKET_Z_MIN = BODY_CENTER_Z - 20.65  # 65.00 mm
POCKET_Z_MAX = BODY_CENTER_Z + 20.65  # 106.30 mm (41.30 mm tall)
POCKET_HALF_Y = 10.20       # 20.40 mm wide
MG996R_SCREW_Z_OFF = 24.5   # 49.0 mm longitudinal pitch -> Z = 61.15 & 110.15 mm
MG996R_SCREW_Y_OFF = 5.0    # 10.0 mm transverse pitch -> Y = +/-5.0 mm
MG996R_PILOT_R = 1.20       # Ø2.4 mm pilot bore (16 mm deep) for M3 / heavy servo mounting screws

# Screw & Guide Rods (GUIDE placed so rods never collide with Ø42 knob at any desk 10-50 mm)
SCREW_X = -6.0
GUIDE_X = -25.0
GUIDE_Y = 19.0
GUIDE_ROD_R = 3.85
GUIDE_BORE_R = 4.2

THREAD_PITCH = 3.0
THREAD_MAJOR_R_NUT = 8.4
THREAD_MINOR_R_NUT = 6.9
THREAD_CLEARANCE = 0.35
THREAD_MAJOR_R_SCREW = THREAD_MAJOR_R_NUT - THREAD_CLEARANCE
THREAD_MINOR_R_SCREW = THREAD_MINOR_R_NUT - THREAD_CLEARANCE

SAMPLE_DESK_T = 13.0        # Positioned clamped onto a 1.3 cm (13 mm) desk!
MJ_T = 14.0
MJ_X_MIN = -30.5
MJ_X_MAX = 17.0
MJ_HALF_W = 26.0
SWIVEL_R = 7.6              # Screw pin stays SWIVEL_R - 0.6 = 7.00 mm (backward compatible!)
SWIVEL_POCKET_R = 7.35      # 0.35 mm radial slip fit on jaw socket (reduces wobble without guide rods)


def make_trap_thread_cutter(major_r, minor_r, pitch, height):
    turns = height / pitch + 2.0
    z_start = -pitch
    helix = Helix(pitch=pitch, height=turns * pitch, radius=minor_r, center=(0, 0, z_start))
    half_base = pitch * 0.40
    half_tip = pitch * 0.16
    inner_r = minor_r - 0.5
    with BuildSketch(Plane.XZ) as sk:
        with BuildLine():
            Polyline(
                (inner_r, z_start - half_base),
                (major_r, z_start - half_tip),
                (major_r, z_start + half_tip),
                (inner_r, z_start + half_base),
                (inner_r, z_start - half_base),
            )
        make_face()
    thread_rib = sweep(sk.sketch, path=helix, is_frenet=True)
    trim_box = Pos(0, 0, height / 2.0) * Box(major_r * 3, major_r * 3, height)
    return thread_rib & trim_box


def build_clamp_body():
    """Monolithic C-clamp with flat reinforced top deck, shear keyway, and 4x M4 nut traps."""
    with BuildPart() as p:
        with Locations((0.5 * (BACK_X_OUTER + JAW_X_FRONT), 0, DECK_Z_TOP / 2.0)):
            Box(JAW_X_FRONT - BACK_X_OUTER, FRAME_HALF_W * 2, DECK_Z_TOP)
        with Locations((0.5 * (BACK_X_OUTER + BACK_X_INNER), 0, 0.5 * (LOWER_JAW_Z_BOT + DECK_Z_TOP))):
            Box(BACK_X_INNER - BACK_X_OUTER, FRAME_HALF_W * 2, DECK_Z_TOP - LOWER_JAW_Z_BOT)
        with Locations((0.5 * (BACK_X_OUTER + JAW_X_FRONT), 0, 0.5 * (LOWER_JAW_Z_BOT + LOWER_JAW_Z_TOP))):
            Box(JAW_X_FRONT - BACK_X_OUTER, FRAME_HALF_W * 2, LOWER_JAW_T)

        gusset_sz = 12.0
        with Locations((BACK_X_INNER + gusset_sz / 2.0, 0, LOWER_JAW_Z_TOP + gusset_sz / 2.0)):
            Wedge(gusset_sz, 20.0, gusset_sz, 0, 0, 0, 0)

        with Locations((0.5 * (KEY_X_MIN + KEY_X_MAX), 0, DECK_Z_TOP - KEY_DEPTH / 2.0)):
            Box(KEY_X_MAX - KEY_X_MIN, KEY_HALF_W * 2, KEY_DEPTH, mode=Mode.SUBTRACT)

        for bx in M4_BOLT_XS:
            for sy in (-1, 1):
                by = sy * M4_BOLT_Y
                with Locations((bx, by, 13.5)):
                    Cylinder(radius=M4_HOLE_R, height=13.0, mode=Mode.SUBTRACT)
                with Locations((bx, by + sy * 3.6, 10.75)):
                    Box(M4_NUT_AF, 14.5, M4_NUT_H, mode=Mode.SUBTRACT)

        with Locations((SCREW_X, 0, 0.5 * (LOWER_JAW_Z_BOT + LOWER_JAW_Z_TOP))):
            Cylinder(radius=THREAD_MINOR_R_NUT, height=LOWER_JAW_T + 2.0, mode=Mode.SUBTRACT)

    nut_cutter = make_trap_thread_cutter(THREAD_MAJOR_R_NUT, THREAD_MINOR_R_NUT, THREAD_PITCH, LOWER_JAW_T)
    nut_cutter = Pos(SCREW_X, 0, LOWER_JAW_Z_BOT) * nut_cutter
    return p.part - nut_cutter


def build_monolithic_joint_pylon():
    """
    Dedicated Solid Base-to-Servo Joint Pylon for TowerPro MG996R:
    - 10 mm thick bolted base flange + bottom shear key (100% compatible with V6 clamp body)
    - Wide flared pedestal + lateral Y-wings cleanly inside all 4x M4 bolt clearances
    - Open-back 29 x 28 mm upper servo head housing MG996R (41.3 x 20.4 mm pocket) at SHAFT_Z=96 mm
    - 4x Ø2.4 mm pilot screw holes (49.0 x 10.0 mm pitch) on HEAD_X_FRONT=-20.5 mm
    - Rear bottom cable relief notch clear of front M3 screw threads
    """
    with BuildPart() as p:
        # 1. Bottom shear key boss (0.25 mm slip fit inside Z=16..19 keyway)
        with Locations((0.5 * (KEY_X_MIN + KEY_X_MAX), 0, DECK_Z_TOP - (KEY_DEPTH - 0.25) / 2.0 + 0.05)):
            Box((KEY_X_MAX - KEY_X_MIN) - 0.5, (KEY_HALF_W * 2) - 0.5, KEY_DEPTH - 0.20)

        # 2. Compact 8 mm mounting base plate (Z=19..27, X=-47..+17, Y=+/-29)
        with Locations((0.5 * (JOINT_X_MIN + JOINT_X_MAX), 0, DECK_Z_TOP + FLANGE_T / 2.0)):
            Box(JOINT_X_MAX - JOINT_X_MIN, JOINT_HALF_W * 2, FLANGE_T)

        # 3. Reinforced low-profile base-to-housing gussets (+X rear & +/-Y sides, Z=27..31.5 mm)
        #    Keeps -X front face (-18.5) 100% flush for MG996R flange + screws, and stays clear of M4 bolts (Y=+/-23).
        with BuildSketch(Plane.YZ.offset(HEAD_X_FRONT)) as sk_side_gusset:
            with BuildLine():
                Polyline(
                    (-HEAD_HALF_Y - 3.3, FLANGE_Z_TOP - 0.5),
                    (HEAD_HALF_Y + 3.3, FLANGE_Z_TOP - 0.5),
                    (HEAD_HALF_Y, FLANGE_Z_TOP + 4.5),
                    (-HEAD_HALF_Y, FLANGE_Z_TOP + 4.5),
                    (-HEAD_HALF_Y - 3.3, FLANGE_Z_TOP - 0.5),
                )
            make_face()
        extrude(sk_side_gusset.sketch, amount=(HEAD_X_BACK - HEAD_X_FRONT))

        with BuildSketch(Plane.XZ) as sk_back_gusset:
            with BuildLine():
                Polyline(
                    (HEAD_X_BACK - 0.5, FLANGE_Z_TOP - 0.5),
                    (min(JOINT_X_MAX, HEAD_X_BACK + 2.5), FLANGE_Z_TOP - 0.5),
                    (HEAD_X_BACK - 0.5, FLANGE_Z_TOP + 4.5),
                    (HEAD_X_BACK - 0.5, FLANGE_Z_TOP - 0.5),
                )
            make_face()
        extrude(sk_back_gusset.sketch, amount=HEAD_HALF_Y, both=True)

        # 5. Monolithic closed-wall MG996R pylon head (X=[-18.5, +14.5], Y=+/-15.5, Z=27.0..77.4 + beveled roof to 85.0)
        head_len_rect = (POCKET_Z_MAX + 0.2) - NECK_START_Z
        with Locations((0.5 * (HEAD_X_FRONT + HEAD_X_BACK), 0, NECK_START_Z + head_len_rect / 2.0)):
            Box(HEAD_X_BACK - HEAD_X_FRONT, HEAD_HALF_Y * 2, head_len_rect)

        with BuildSketch(Plane.YZ.offset(HEAD_X_FRONT)) as sk_roof:
            with BuildLine():
                Polyline(
                    (-HEAD_HALF_Y, POCKET_Z_MAX + 0.1),
                    (HEAD_HALF_Y, POCKET_Z_MAX + 0.1),
                    (HEAD_TOP_Y, HEAD_TOP_Z),
                    (-HEAD_TOP_Y, HEAD_TOP_Z),
                    (-HEAD_HALF_Y, POCKET_Z_MAX + 0.1),
                )
            make_face()
        extrude(sk_roof.sketch, amount=(HEAD_X_BACK - HEAD_X_FRONT))

        # 5b. Coaxial +X bearing inner-race spacer boss (X=+14.5..+15.5, Y=0, Z=SHAFT_Z=67.0)
        #     Supports 683ZZ (3x8x4) bearing inner race on widened Cover<2> +Y arm (+15.7..+20.2)
        with Locations((HEAD_X_BACK + 0.5 * (BEARING_BOSS_LEN - 0.5), 0, SHAFT_Z)):
            Cylinder(radius=2.75, height=BEARING_BOSS_LEN + 0.5, rotation=(0, 90, 0))

        # --- SUBTRACTIONS ---
        # 6. 4x M4 clearance holes + full-height unobstructed socket-head access tunnels
        for bx in M4_BOLT_XS:
            for sy in (-1, 1):
                by = sy * M4_BOLT_Y
                with Locations((bx, by, DECK_Z_TOP + FLANGE_T / 2.0)):
                    Cylinder(radius=M4_HOLE_R, height=FLANGE_T + 1.0, mode=Mode.SUBTRACT)
                with Locations((bx, by, 0.5 * ((FLANGE_Z_TOP - 3.5) + 140.0))):
                    Cylinder(radius=M4_CBORE_R + 0.1, height=140.0 - (FLANGE_Z_TOP - 3.5), mode=Mode.SUBTRACT)

        # 7. Closed 4-wall MG996R pocket from -X front (X=-19.5..+9.5, Y=+/-10.20, Z=36.00..77.30)
        #    Leaves 5.3 mm solid +/-Y walls & 5.0 mm solid rear bearing bridge wall (+9.5..+14.5)!
        pocket_depth = POCKET_X_BACK - (HEAD_X_FRONT - 1.0)
        with Locations((0.5 * ((HEAD_X_FRONT - 1.0) + POCKET_X_BACK), 0, 0.5 * (POCKET_Z_MIN + POCKET_Z_MAX))):
            Box(pocket_depth, POCKET_HALF_Y * 2, POCKET_Z_MAX - POCKET_Z_MIN, mode=Mode.SUBTRACT)

        # 7b. Center V-arch boot relief under front roof (X=-19.5..-5.0, Y=+/-3.4 -> +/-2.0, Z=77.2..81.5)
        #     Gives 45.5 mm total height down centerline so protruding wire boot slides straight through cleanly,
        #     while staying >=1.5 mm clear of the Y=+/-5.0 M3 screw bores!
        with BuildSketch(Plane.YZ.offset(HEAD_X_FRONT - 1.0)) as sk_arch:
            with BuildLine():
                Polyline(
                    (-3.4, POCKET_Z_MAX - 0.2),
                    (3.4, POCKET_Z_MAX - 0.2),
                    (2.0, 81.5),
                    (-2.0, 81.5),
                    (-3.4, POCKET_Z_MAX - 0.2),
                )
            make_face()
        extrude(sk_arch.sketch, amount=((-5.0) - (HEAD_X_FRONT - 1.0)), mode=Mode.SUBTRACT)

        # 8. 4x MG996R pilot mounting holes on HEAD_X_FRONT (-18.5): 49.0 x 10.0 mm pitch, 12.5 mm deep
        for sz_off in (-MG996R_SCREW_Z_OFF, MG996R_SCREW_Z_OFF):
            for sy_off in (-MG996R_SCREW_Y_OFF, MG996R_SCREW_Y_OFF):
                with Locations((HEAD_X_FRONT + 6.25, sy_off, BODY_CENTER_Z + sz_off)):
                    Cylinder(radius=MG996R_PILOT_R, height=12.7, rotation=(0, 90, 0), mode=Mode.SUBTRACT)

        # 9. Larger top-side wire & boot bay for Waveshare WS-MG996R:
        #    Open out top (Z=75.5..86.0) and -Y side (Y=-16.0..+6.5) across X=-5.5..+10.0 (15.5 x 22.0 mm bay!):
        #    - Shortens front M3 screw bridge to X=-18.5..-5.5 (13.0 mm solid; clears 12.5 mm M3 bores)
        #    - Keeps reinforced +X rear bearing bridge (+10.0..+14.5 = 4.5..5.0 mm) & +Y top beam (+6.5..+15.5 = 9.0 mm) solid!
        slot_z_min = POCKET_Z_MAX - 1.8
        slot_z_max = HEAD_TOP_Z + 1.0
        with Locations((0.5 * (-5.5 + POCKET_X_BACK + 0.5), 0.5 * (-16.0 + 6.5), 0.5 * (slot_z_min + slot_z_max))):
            Box((POCKET_X_BACK + 0.5) - (-5.5), 6.5 - (-16.0), slot_z_max - slot_z_min, mode=Mode.SUBTRACT)

        # 10. Coaxial M3 pilot bore in +X bearing boss (X=+9.4..+15.6 -> 6 mm solid thread engagement!)
        with Locations((0.5 * (POCKET_X_BACK + HEAD_X_BACK + BEARING_BOSS_LEN), 0, SHAFT_Z)):
            Cylinder(radius=1.30, height=(HEAD_X_BACK + BEARING_BOSS_LEN - POCKET_X_BACK) + 0.4, rotation=(0, 90, 0), mode=Mode.SUBTRACT)

    return p.part


def build_moving_jaw(z_top=-SAMPLE_DESK_T):
    z_bot = z_top - MJ_T
    rod_len = (z_bot - LOWER_JAW_Z_BOT) + 3.0  # Extends 3 mm through lower jaw at 13 mm desk (still engages >=10 mm desk!)
    with BuildPart() as p:
        with Locations((0.5 * (MJ_X_MIN + MJ_X_MAX), 0, z_top - MJ_T / 2.0)):
            Box(MJ_X_MAX - MJ_X_MIN, MJ_HALF_W * 2, MJ_T)
        for gx in range(-25, 15, 6):
            with Locations((gx, 0, z_top - 0.4)):
                Box(2.0, MJ_HALF_W * 2 - 6.0, 1.0, mode=Mode.SUBTRACT)
        with Locations((SCREW_X, 0, z_bot + 2.5)):
            Cylinder(radius=SWIVEL_POCKET_R, height=5.5, mode=Mode.SUBTRACT)
    return p.part


def build_clamp_screw(z_jaw_top=-SAMPLE_DESK_T):
    z_tip = z_jaw_top - MJ_T + 4.5
    z_knob_top = LOWER_JAW_Z_BOT - 13.0  # -97 mm -> z_thread_start=-96 mm (-12 mm = -4*3 mm exact pitch sync to -84 mm!)
    knob_h = 15.0
    knob_r = 21.0
    shaft_len = z_tip - z_knob_top
    swivel_neck = 5.0
    with BuildPart() as p:
        with Locations((SCREW_X, 0, z_tip - swivel_neck / 2.0)):
            Cylinder(radius=SWIVEL_R - 0.6, height=swivel_neck)
        thread_h = shaft_len - swivel_neck
        with Locations((SCREW_X, 0, z_knob_top + thread_h / 2.0)):
            Cylinder(radius=THREAD_MINOR_R_SCREW, height=thread_h)
        with Locations((SCREW_X, 0, z_knob_top - knob_h / 2.0)):
            Cylinder(radius=knob_r, height=knob_h)
            for k in range(8):
                ang = k * 45.0
                with Locations((knob_r * math.cos(math.radians(ang)), knob_r * math.sin(math.radians(ang)), 0)):
                    Cylinder(radius=3.5, height=knob_h + 1.0, mode=Mode.SUBTRACT)
    ext_h = thread_h - 1.5
    ext_thread = make_trap_thread_cutter(THREAD_MAJOR_R_SCREW, THREAD_MINOR_R_SCREW, THREAD_PITCH, ext_h)
    ext_thread = Pos(SCREW_X, 0, z_knob_top + 1.0) * ext_thread
    return p.part + ext_thread


def main():
    print("Generating V6 C-Clamp + Dedicated Monolithic Standing Base Joint Pylon...")
    body = build_clamp_body()
    joint = build_monolithic_joint_pylon()
    jaw = build_moving_jaw()
    screw = build_clamp_screw()

    files = {
        "arm_base_v6_body": body,
        "arm_base_v6_joint": joint,
        "arm_base_v6_jaw": jaw,
        "arm_base_v6_screw": screw,
    }
    for name, part in files.items():
        export_step(part, os.path.join(OUT_DIR, f"{name}.step"))
        export_stl(part, os.path.join(OUT_DIR, f"{name}.stl"))
        bb = part.bounding_box()
        print(f"  {name}: X=[{bb.min.X:.2f},{bb.max.X:.2f}], Y=[{bb.min.Y:.2f},{bb.max.Y:.2f}], Z=[{bb.min.Z:.2f},{bb.max.Z:.2f}]")

    asm_compound = Compound(children=[body, joint, jaw, screw])
    export_step(asm_compound, os.path.join(OUT_DIR, "arm_base_v6_complete.step"))
    print("Exported V6 STEPs & STLs.")


if __name__ == "__main__":
    main()

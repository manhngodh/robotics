#!/usr/bin/env python3
"""
Modular C-Clamp Base + Dedicated Heavy-Duty Standing Base Joint (V5) for M Robot / MD Station.

Orientation: Cover<1> stands vertically (+Z) via Ry(-90) at pos=(+0.5, 0.0, 49.0),
putting its rear clevis pivot squarely over the clamp center at (-15.0, 0.0, 43.0) mm
and standing the whole arm upright along +Z instead of leaning horizontally 90 deg.

Parts:
1. arm_base_v5_body: Monolithic C-clamp frame (15-55 mm desk, 50 mm throat) with flat top
   deck (Z=19 mm), 36x32x3 mm shear keyway, and 4x M4 side-sliding nut traps.
2. arm_base_v5_joint: Dedicated heavy-duty Standing Base Joint bolted to clamp deck (4x M4
   + bottom shear key), with thick vertical riser tower inside Cover<1> U-slot (up to Z=63.6),
   +X/+X vertical stabilizing buttresses, dual 6.65 mm outer clevis cheeks at Y=+/-(22.35..29.0),
   and native clevis fastener interfaces at (-15.0, 0, 43.0) (-Y: Ø4.55 spigot + M2.5; +Y: M3/8mm bearing).
3. arm_base_v5_jaw: Anti-rotation moving jaw with dual Ø7.7 mm guide rods & swivel socket.
4. arm_base_v5_screw: 3D-printable Tr16x3 leadscrew + 8-lobe ergonomic knob (pitch-synced).
"""

import os
import math
from build123d import *

OUT_DIR = "/Users/manhngodh/development/robotics"

# ====================================================================
# 1. COORDINATES (Cover<1> standing vertical via Ry(-90) + t=[+0.5, 0.0, 49.0])
#    Cover<1> pivot hole: X = -15.0, Y = 0.0, Z = 43.0
#    Cover<1> bottom tip (X_loc=-14.5): Z = 34.5 mm
#    Cover<1> bridge underside (X_loc=+15.0): Z = 64.0 mm
#    Cover<1> X envelope along clevis (Z=34.5..64.0): X = [-23.5, +0.5]
#    Desk top at Z = 0.0 mm.
# ====================================================================
BACK_X_OUTER = -48.0
BACK_X_INNER = -32.0
JAW_X_FRONT = 18.0
DECK_Z_TOP = 19.0
FRAME_HALF_W = 30.25        # 60.5 mm wide

THROAT_OPENING = 82.0       # Clamps 15-55 mm desk cleanly
LOWER_JAW_T = 18.0
LOWER_JAW_Z_TOP = -THROAT_OPENING
LOWER_JAW_Z_BOT = LOWER_JAW_Z_TOP - LOWER_JAW_T  # -100.0 mm

# Shear Keyway + 4x M4 Bolt Pattern on Z=19.0 Deck
KEY_X_MIN = -36.0
KEY_X_MAX = +6.0
KEY_HALF_W = 16.0
KEY_DEPTH = 3.0

M4_BOLT_XS = [-41.0, 11.0]  # Outside the standing joint tower/cheeks (100% open from top)
M4_BOLT_Y = 23.0
M4_HOLE_R = 2.2             # Ø4.4 mm clearance
M4_CBORE_R = 3.9            # Ø7.8 mm socket head counterbore
M4_NUT_AF = 7.2
M4_NUT_H = 3.5

# Dedicated Standing Base Joint Adapter (Z=19.0..63.6)
JOINT_X_MIN = -47.0
JOINT_X_MAX = 17.0          # 64 mm long base footprint
JOINT_HALF_W = 29.0         # 58 mm wide
FLANGE_T = 9.0              # 9 mm thick mounting base plate (Z=19.0..28.0)
FLANGE_Z_TOP = DECK_Z_TOP + FLANGE_T  # 28.0 mm (6.5 mm clear below Cover<1> bottom Z=34.5)

PIVOT_X = -15.0
PIVOT_Z = 43.0
ARM_HALF_W = 22.0
SLOT_HALF_W = 17.5
TONGUE_HALF_W = SLOT_HALF_W - 0.40  # 17.10 mm (34.2 mm wide)
CHEEK_INNER_Y = ARM_HALF_W + 0.35   # 22.35 mm
CHEEK_OUTER_Y = JOINT_HALF_W        # 29.00 mm (6.65 mm wall)

# Screw & Guide Rods
SCREW_X = -6.0
GUIDE_X = -22.0
GUIDE_Y = 16.0
GUIDE_ROD_R = 3.85
GUIDE_BORE_R = 4.2

THREAD_PITCH = 3.0
THREAD_MAJOR_R_NUT = 8.4
THREAD_MINOR_R_NUT = 6.9
THREAD_CLEARANCE = 0.35
THREAD_MAJOR_R_SCREW = THREAD_MAJOR_R_NUT - THREAD_CLEARANCE
THREAD_MINOR_R_SCREW = THREAD_MINOR_R_NUT - THREAD_CLEARANCE

SAMPLE_DESK_T = 30.0
MJ_T = 14.0
MJ_X_MIN = -29.0
MJ_X_MAX = 17.0
MJ_HALF_W = 26.0
SWIVEL_R = 7.6


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
    """Monolithic C-clamp with flat reinforced top deck, centered shear keyway, and 4x M4 nut traps."""
    with BuildPart() as p:
        with Locations((0.5 * (BACK_X_OUTER + JAW_X_FRONT), 0, DECK_Z_TOP / 2.0)):
            Box(JAW_X_FRONT - BACK_X_OUTER, FRAME_HALF_W * 2, DECK_Z_TOP)

        with Locations((0.5 * (BACK_X_OUTER + BACK_X_INNER), 0, 0.5 * (LOWER_JAW_Z_BOT + DECK_Z_TOP))):
            Box(BACK_X_INNER - BACK_X_OUTER, FRAME_HALF_W * 2, DECK_Z_TOP - LOWER_JAW_Z_BOT)

        with Locations((0.5 * (BACK_X_OUTER + JAW_X_FRONT), 0, 0.5 * (LOWER_JAW_Z_BOT + LOWER_JAW_Z_TOP))):
            Box(JAW_X_FRONT - BACK_X_OUTER, FRAME_HALF_W * 2, LOWER_JAW_T)

        # Center lower jaw gusset (between dual guide rods at Y=+/-16, below max 55mm desk jaw Z=-69)
        gusset_sz = 12.0
        with Locations((BACK_X_INNER + gusset_sz / 2.0, 0, LOWER_JAW_Z_TOP + gusset_sz / 2.0)):
            Wedge(gusset_sz, 20.0, gusset_sz, 0, 0, 0, 0)

        # Shear keyway pocket on top deck (Z=16..19)
        with Locations((0.5 * (KEY_X_MIN + KEY_X_MAX), 0, DECK_Z_TOP - KEY_DEPTH / 2.0)):
            Box(KEY_X_MAX - KEY_X_MIN, KEY_HALF_W * 2, KEY_DEPTH, mode=Mode.SUBTRACT)

        # 4x M4 vertical bores + side-accessible captive M4 hex nut slots
        for bx in M4_BOLT_XS:
            for sy in (-1, 1):
                by = sy * M4_BOLT_Y
                with Locations((bx, by, 13.5)):
                    Cylinder(radius=M4_HOLE_R, height=13.0, mode=Mode.SUBTRACT)
                with Locations((bx, by + sy * 3.6, 10.75)):
                    Box(M4_NUT_AF, 14.5, M4_NUT_H, mode=Mode.SUBTRACT)

        for sy in (-1, 1):
            with Locations((GUIDE_X, sy * GUIDE_Y, 0.5 * (LOWER_JAW_Z_BOT + LOWER_JAW_Z_TOP))):
                Cylinder(radius=GUIDE_BORE_R, height=LOWER_JAW_T + 2.0, mode=Mode.SUBTRACT)

        with Locations((SCREW_X, 0, 0.5 * (LOWER_JAW_Z_BOT + LOWER_JAW_Z_TOP))):
            Cylinder(radius=THREAD_MINOR_R_NUT, height=LOWER_JAW_T + 2.0, mode=Mode.SUBTRACT)

    nut_cutter = make_trap_thread_cutter(THREAD_MAJOR_R_NUT, THREAD_MINOR_R_NUT, THREAD_PITCH, LOWER_JAW_T)
    nut_cutter = Pos(SCREW_X, 0, LOWER_JAW_Z_BOT) * nut_cutter
    return p.part - nut_cutter


def build_base_joint():
    """
    Dedicated Heavy-Duty Standing Base Joint Adapter:
    - 9 mm thick bolted flange + centered bottom shear key (Z=16.25..28.0)
    - Standing vertical tower (Y=+/-17.10, Z=28.0..63.6) filling Cover<1>'s U-slot
      right up to 0.4 mm below its bridge underside (Z=64.0)
    - Front & rear buttress cradles + vertical stop cheeks (X=-23.9..+0.8) stopping nod/flex
    - Dual 6.65 mm thick outer clevis cheeks at Y=+/-(22.35..29.0) with flared gussets
    - Cross-axis hardware at (-15.0, 0.0, 43.0): -Y Ø4.55 spigot + M2.5, +Y M3 / 8mm bearing
    """
    with BuildPart() as p:
        # 1. Bottom shear key boss (0.25 mm slip fit inside Z=16..19 keyway)
        with Locations((0.5 * (KEY_X_MIN + KEY_X_MAX), 0, DECK_Z_TOP - (KEY_DEPTH - 0.25) / 2.0 + 0.05)):
            Box((KEY_X_MAX - KEY_X_MIN) - 0.5, (KEY_HALF_W * 2) - 0.5, KEY_DEPTH - 0.20)

        # 2. Heavy 9 mm mounting base plate (Z=19.0..28.0, X=-47..17, Y=+/-29)
        with Locations((0.5 * (JOINT_X_MIN + JOINT_X_MAX), 0, DECK_Z_TOP + FLANGE_T / 2.0)):
            Box(JOINT_X_MAX - JOINT_X_MIN, JOINT_HALF_W * 2, FLANGE_T)

        # 3. Standing central riser tower inside Cover<1> U-slot (Y=+/-17.10, Z=28.0..63.6)
        #    Flared pedestal base (X=-34..+4 at Z=28) rising to vertical clevis spine (X=-20.3..+0.3 at Z=63.6)
        with BuildSketch(Plane.XZ) as sk_tower:
            with BuildLine():
                Polyline(
                    (-34.0, FLANGE_Z_TOP - 0.5),
                    (6.5, FLANGE_Z_TOP - 0.5),
                    (6.5, 34.0),
                    (0.3, 63.6),
                    (-20.3, 63.6),
                    (-23.2, 43.0),
                    (-34.0, FLANGE_Z_TOP - 0.5),
                )
            make_face()
        extrude(sk_tower.sketch, amount=TONGUE_HALF_W, both=True)

        # 4. Under-clevis cradle saddle & +X / -X vertical backstops across Y=+/-22.0
        #    - Saddle directly beneath Cover<1> bottom tip (Z=28.0..34.1, 0.4 mm clear of Z=34.5)
        with Locations((-15.0, 0, 0.5 * (FLANGE_Z_TOP + 34.1))):
            Box(18.0, ARM_HALF_W * 2, 34.1 - FLANGE_Z_TOP)
        #    - Vertical +X backstop rib behind Cover<1> (X=+0.9..+6.5, Z=28.0..62.0, Y=+/-22.0)
        with BuildSketch(Plane.XZ) as sk_back:
            with BuildLine():
                Polyline(
                    (0.9, FLANGE_Z_TOP - 0.5),
                    (6.8, FLANGE_Z_TOP - 0.5),
                    (6.8, 34.0),
                    (4.5, 62.0),
                    (0.9, 62.0),
                    (0.9, FLANGE_Z_TOP - 0.5),
                )
            make_face()
        extrude(sk_back.sketch, amount=ARM_HALF_W, both=True)
        #    - Vertical -X front stop rib ahead of Cover<1> (X=-29.0..-23.9, Z=28.0..58.0, Y=+/-22.0)
        with BuildSketch(Plane.XZ) as sk_front:
            with BuildLine():
                Polyline(
                    (-34.0, FLANGE_Z_TOP - 0.5),
                    (-23.9, FLANGE_Z_TOP - 0.5),
                    (-23.9, 58.0),
                    (-27.0, 58.0),
                    (-34.0, FLANGE_Z_TOP - 0.5),
                )
            make_face()
        extrude(sk_front.sketch, amount=ARM_HALF_W, both=True)

        # 5. Dual heavy outer standing clevis cheeks (Y=+/-(22.35..29.0)) + flared base gussets
        #    Bounds along X stay cleanly within [-35, +5] so M4 bolts at X=[-41, +11] are 100% open!
        cheek_t = CHEEK_OUTER_Y - CHEEK_INNER_Y
        for sy in (-1, 1):
            y_mid = sy * 0.5 * (CHEEK_INNER_Y + CHEEK_OUTER_Y)
            with BuildSketch(Plane.XZ.offset(y_mid)) as sk_cheek:
                with BuildLine():
                    Polyline(
                        (-35.0, FLANGE_Z_TOP - 0.5),
                        (6.5, FLANGE_Z_TOP - 0.5),
                        (6.5, 34.0),
                        (1.5, 58.0),
                        (-15.0, 59.5),
                        (-27.0, 56.0),
                        (-35.0, FLANGE_Z_TOP - 0.5),
                    )
                make_face()
            extrude(sk_cheek.sketch, amount=cheek_t / 2.0, both=True)

        # 6. Native clevis bosses at (-15.0, 0.0, 43.0):
        #    -Y side: Ø4.55 mm spigot protruding 2.1 mm from tongue (-17.10 -> -19.20)
        with Locations((PIVOT_X, -TONGUE_HALF_W - 1.05, PIVOT_Z)):
            Cylinder(radius=2.275, height=2.2, rotation=(90, 0, 0))
        #    +Y side: Ø5.5 mm bearing inner-race spacer boss protruding 0.35 mm from +Y cheek (22.35 -> 22.00)
        with Locations((PIVOT_X, CHEEK_INNER_Y - 0.175, PIVOT_Z)):
            Cylinder(radius=2.75, height=0.45, rotation=(90, 0, 0))

        # --- SUBTRACTIONS ---
        # 7. 4x M4 clearance holes + top counterbores
        for bx in M4_BOLT_XS:
            for sy in (-1, 1):
                by = sy * M4_BOLT_Y
                with Locations((bx, by, DECK_Z_TOP + FLANGE_T / 2.0)):
                    Cylinder(radius=M4_HOLE_R, height=FLANGE_T + 1.0, mode=Mode.SUBTRACT)
                with Locations((bx, by, FLANGE_Z_TOP - 2.25)):
                    Cylinder(radius=M4_CBORE_R, height=4.6, mode=Mode.SUBTRACT)

        # 8. Cross-axis fastener bores & counterbores at (-15.0, 0.0, 43.0):
        with Locations((PIVOT_X, -14.75, PIVOT_Z)):
            Cylinder(radius=1.35, height=29.5, rotation=(90, 0, 0), mode=Mode.SUBTRACT)
        with Locations((PIVOT_X, -CHEEK_OUTER_Y + 1.6, PIVOT_Z)):
            Cylinder(radius=2.70, height=3.4, rotation=(90, 0, 0), mode=Mode.SUBTRACT)
        with Locations((PIVOT_X, 14.75, PIVOT_Z)):
            Cylinder(radius=1.65, height=29.5, rotation=(90, 0, 0), mode=Mode.SUBTRACT)
        with Locations((PIVOT_X, CHEEK_OUTER_Y - 1.6, PIVOT_Z)):
            Cylinder(radius=3.00, height=3.4, rotation=(90, 0, 0), mode=Mode.SUBTRACT)

        # 9. Vertical nut-drop slots from top of tower (Z=63.6 down to Z=40.0) for M2.5 (-Y) and M3 (+Y)
        with Locations((PIVOT_X, -10.0, 52.0)):
            Box(5.2, 2.5, 24.5, mode=Mode.SUBTRACT)
        with Locations((PIVOT_X, 10.0, 52.0)):
            Box(5.7, 3.0, 24.5, mode=Mode.SUBTRACT)

    return p.part


def build_moving_jaw(z_top=-SAMPLE_DESK_T):
    z_bot = z_top - MJ_T
    rod_len = (z_bot - LOWER_JAW_Z_BOT) + 2.0
    with BuildPart() as p:
        with Locations((0.5 * (MJ_X_MIN + MJ_X_MAX), 0, z_top - MJ_T / 2.0)):
            Box(MJ_X_MAX - MJ_X_MIN, MJ_HALF_W * 2, MJ_T)
        for gx in range(-24, 15, 6):
            with Locations((gx, 0, z_top - 0.4)):
                Box(2.0, MJ_HALF_W * 2 - 6.0, 1.0, mode=Mode.SUBTRACT)
        for sy in (-1, 1):
            with Locations((GUIDE_X, sy * GUIDE_Y, z_bot - rod_len / 2.0)):
                Cylinder(radius=GUIDE_ROD_R, height=rod_len)
        with Locations((SCREW_X, 0, z_bot + 2.5)):
            Cylinder(radius=SWIVEL_R, height=5.5, mode=Mode.SUBTRACT)
    return p.part


def build_clamp_screw(z_jaw_top=-SAMPLE_DESK_T):
    z_tip = z_jaw_top - MJ_T + 4.5
    z_knob_top = LOWER_JAW_Z_BOT - 25.0  # Exact 8x3 mm pitch sync from LOWER_JAW_Z_BOT (-100)
    knob_h = 15.0
    knob_r = 21.0
    shaft_len = z_tip - z_knob_top
    with BuildPart() as p:
        with Locations((SCREW_X, 0, z_tip - 4.0)):
            Cylinder(radius=SWIVEL_R - 0.6, height=8.0)
        thread_h = shaft_len - 8.0
        with Locations((SCREW_X, 0, z_knob_top + thread_h / 2.0)):
            Cylinder(radius=THREAD_MINOR_R_SCREW, height=thread_h)
        with Locations((SCREW_X, 0, z_knob_top - knob_h / 2.0)):
            Cylinder(radius=knob_r, height=knob_h)
            for k in range(8):
                ang = k * 45.0
                with Locations((knob_r * math.cos(math.radians(ang)), knob_r * math.sin(math.radians(ang)), 0)):
                    Cylinder(radius=3.5, height=knob_h + 1.0, mode=Mode.SUBTRACT)
    ext_h = thread_h - 2.0
    ext_thread = make_trap_thread_cutter(THREAD_MAJOR_R_SCREW, THREAD_MINOR_R_SCREW, THREAD_PITCH, ext_h)
    ext_thread = Pos(SCREW_X, 0, z_knob_top + 1.0) * ext_thread
    return p.part + ext_thread


def main():
    print("Generating V5 Standing Modular C-Clamp + Heavy Base Joint...")
    body = build_clamp_body()
    joint = build_base_joint()
    jaw = build_moving_jaw()
    screw = build_clamp_screw()

    files = {
        "arm_base_v5_body": body,
        "arm_base_v5_joint": joint,
        "arm_base_v5_jaw": jaw,
        "arm_base_v5_screw": screw,
    }
    for name, part in files.items():
        export_step(part, os.path.join(OUT_DIR, f"{name}.step"))
        export_stl(part, os.path.join(OUT_DIR, f"{name}.stl"))
        bb = part.bounding_box()
        print(f"  {name}: X=[{bb.min.X:.2f},{bb.max.X:.2f}], Y=[{bb.min.Y:.2f},{bb.max.Y:.2f}], Z=[{bb.min.Z:.2f},{bb.max.Z:.2f}]")

    asm_compound = Compound(children=[body, joint, jaw, screw])
    export_step(asm_compound, os.path.join(OUT_DIR, "arm_base_v5_complete.step"))
    print("Exported standing V5 STEPs & STLs.")


if __name__ == "__main__":
    main()

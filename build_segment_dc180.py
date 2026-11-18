#!/usr/bin/env python3
"""
Heavy-Duty Rotary Drill End-Effector for Snake Robot Arm.
Features:
- 100% genuine replacement U-fork from Onshape Tab 4.
- 10.5mm spacious wiring chamber with zip-tie strain-relief anchor.
- Heavy-duty 4x M3 clamping system (front & rear pairs) with captive hex nut pockets.
- 4.0mm thick reinforced front spindle faceplate with Ø11.5mm chuck aperture and 2x M2 locking screws.
- Dual ventilation slots (top & bottom) for continuous drill cooling.
- 100% Support-free FDM 3D printing plate layout.
- Automated push to Onshape.
"""

import os
import json
import time
from build123d import *
from onshape_client.client import Client

# ====================================================================
# 1. PARAMETERS
# ====================================================================
# DC 180 Motor specs
motor_w = 20.4        # Motor width (cylindrical diameter) [mm]
motor_t = 15.4        # Motor thickness (between flats) [mm]
motor_l = 32.0        # Motor metal body length [mm]
clearance = 0.25      # 3D printing fit clearance [mm]

r_c = (motor_w + 2 * clearance) / 2  # 10.45 mm
h_c = (motor_t + 2 * clearance) / 2  # 7.95 mm

z_axis = 15.0         # Motor shaft centerline height [mm]
x_start = 19.5        # Interface with U-fork bridge [mm]
x_motor_start = 30.0  # Motor body start (châm 10.5mm wiring chamber) [mm]
x_motor_end = 62.0    # Motor body end (32.0 mm length) [mm]
x_end = 66.0          # Front spindle faceplate end (4.0 mm thickness) [mm]
cradle_len = x_end - x_start  # 46.5 mm

body_w = 34.0         # Cradle width across ears [mm]
bolt_y = 13.5         # M3 bolt spacing across Y (+/- 13.5 mm)
bolt_x1 = 38.0        # Rear M3 bolt pair [mm]
bolt_x2 = 55.0        # Front M3 bolt pair [mm]
clamp_gap = 1.0       # Clamping gap between cap & cradle ears [mm]
ear_h = 5.6           # Ear thickness [mm]
cap_top_z = z_axis + h_c + 3.2  # Top crown of cap (26.15 mm)

cap_start = 29.0      # Cap front edge [mm]
cap_end = 64.5        # Cap rear edge [mm]
cap_len = cap_end - cap_start

# ====================================================================
# 2. LOAD & FUSE REPLACEMENT U-FORK
# ====================================================================
print("1. Loading replacement U-fork from Tab 4...")
tab4_step = "/Users/manhngodh/development/onshape/02_universal_segment_tab4.step"
part = import_step(tab4_step)
fork_solids = [part.solids()[0], part.solids()[1], part.solids()[3]]
fork = fork_solids[0]
for s in fork_solids[1:]:
    fork = fork.fuse(s)
print(f"   Fork loaded! Volume: {fork.volume:.1f} mm^3")

# ====================================================================
# 3. BUILD REINFORCED DC 180 CRADLE
# ====================================================================
print("2. Building reinforced DC 180 cradle with wiring chamber...")
with BuildPart() as cradle_builder:
    # Outer block with smooth rounded edges
    with BuildSketch(Plane.YZ.offset(x_start)) as s_cradle:
        with Locations([(0, z_axis / 2)]):
            RectangleRounded(body_w, z_axis, radius=3.0)
    extrude(s_cradle.sketch, amount=cradle_len)

    # Motor cavity cut (lower half)
    plane_yz_motor = Plane(origin=(x_motor_start, 0, z_axis), x_dir=(0, 1, 0), z_dir=(1, 0, 0))
    with BuildSketch(plane_yz_motor) as s_cavity:
        Circle(radius=r_c)
        Rectangle(r_c * 2 + 5, h_c * 2, mode=Mode.INTERSECT)
    extrude(s_cavity.sketch, amount=x_motor_end - x_motor_start, mode=Mode.SUBTRACT)

    # Spacious Wiring Chamber (X in [19.5, 30.0], Y in [-8.5, 8.5], Z in [4.0, z_axis+1])
    with BuildSketch(Plane.XY.offset(4.0)) as s_chamber:
        with Locations([( (x_start + x_motor_start)/2, 0 )]):
            Rectangle(x_motor_start - x_start + 1.0, 17.0)
    extrude(s_chamber.sketch, amount=z_axis - 4.0 + 2.0, mode=Mode.SUBTRACT)

    # Zip-tie anchor slot (through-tunnel for zip-tie under wire bed at X = 24.5)
    with BuildSketch(Plane.XZ.offset(-8.5)) as s_ziptie:
        with Locations([(24.5, 2.2)]):
            RectangleRounded(3.0, 2.0, radius=0.8)
    extrude(s_ziptie.sketch, amount=17.0, mode=Mode.SUBTRACT)

    # Front Spindle Faceplate (X in [62.0, 66.0]):
    # 1. Spindle chuck opening Ø11.5mm
    with BuildSketch(Plane.YZ.offset(x_motor_end - 0.5)) as s_chuck:
        with Locations([(0, z_axis)]):
            Circle(radius=11.5 / 2)
    extrude(s_chuck.sketch, amount=x_end - x_motor_end + 1.0, mode=Mode.SUBTRACT)

    # 2. 2x M2 front screws (pitch 10mm, Y = +/- 5mm) with counterbores
    for sy in [-1, 1]:
        with Locations([(x_motor_end - 0.5, sy * 5.0, z_axis)]):
            with BuildSketch(Plane.YZ) as s_m2:
                Circle(radius=2.4 / 2)
            extrude(s_m2.sketch, amount=x_end - x_motor_end + 1.0, mode=Mode.SUBTRACT)
        # Counterbore on front face for M2 screw heads
        with Locations([(x_end - 2.0, sy * 5.0, z_axis)]):
            with BuildSketch(Plane.YZ) as s_cb:
                Circle(radius=4.5 / 2)
            extrude(s_cb.sketch, amount=3.0, mode=Mode.SUBTRACT)

    # 4x M3 clamping bolt holes & bottom hex nut pockets
    for bx in [bolt_x1, bolt_x2]:
        for sy in [-1, 1]:
            with Locations([(bx, sy * bolt_y, -1.0)]):
                Cylinder(radius=3.4 / 2, height=z_axis + 2.0, mode=Mode.SUBTRACT)
            with Locations([(bx, sy * bolt_y, 0)]):
                with BuildSketch() as s_nut:
                    RegularPolygon(radius=5.85 / (3**0.5), side_count=6)
                extrude(s_nut.sketch, amount=3.5, mode=Mode.SUBTRACT)

    # Bottom vent slot (under motor)
    with BuildSketch(Plane.XY.offset(-1.0)) as s_vent:
        with Locations([( (x_motor_start + x_motor_end)/2, 0 )]):
            Rectangle(18.0, 7.0)
    extrude(s_vent.sketch, amount=z_axis + 2.0, mode=Mode.SUBTRACT)

cradle = cradle_builder.part

# Fuse fork and cradle into monolithic body
full_segment_cradle = fork.fuse(cradle)
print(f"   Monolithic cradle fused! Volume: {full_segment_cradle.volume:.1f} mm^3")

# ====================================================================
# 4. BUILD MATCHING REINFORCED TOP CLAMP CAP (4 BOLTS)
# ====================================================================
print("3. Building heavy-duty 4-bolt top clamp cap...")
with BuildPart() as cap_builder:
    # Outer hull: base block at ears level
    with BuildSketch(Plane.YZ.offset(cap_start)) as s_ear:
        with Locations([(0, z_axis + clamp_gap + ear_h / 2)]):
            RectangleRounded(body_w, ear_h, radius=1.8)
    b_ear = extrude(s_ear.sketch, amount=cap_len)

    # Top arch crown block
    with BuildSketch(Plane.YZ.offset(cap_start)) as s_top:
        with Locations([(0, cap_top_z - 1.5)]):
            RectangleRounded(r_c * 1.5, 3.0, radius=1.0)
    b_top = extrude(s_top.sketch, amount=cap_len)

    # Loft between ears and crown
    loft([s_ear.sketch, s_top.sketch])

    # Motor cavity cut (upper half)
    with BuildSketch(plane_yz_motor) as s_cap_cavity:
        Circle(radius=r_c)
        Rectangle(r_c * 2 + 5, h_c * 2, mode=Mode.INTERSECT)
    extrude(s_cap_cavity.sketch, amount=x_motor_end - x_motor_start, mode=Mode.SUBTRACT)

    # Front chuck aperture 11.5mm
    with BuildSketch(Plane.YZ.offset(x_motor_end - 0.5)) as s_front:
        with Locations([(0, z_axis)]):
            Circle(radius=11.5 / 2)
    extrude(s_front.sketch, amount=cap_end - x_motor_end + 1.0, mode=Mode.SUBTRACT)

    # Top vent slot
    with BuildSketch(Plane.XY.offset(z_axis)) as s_top_vent:
        with Locations([( (x_motor_start + x_motor_end)/2, 0 )]):
            Rectangle(18.0, 7.0)
    extrude(s_top_vent.sketch, amount=20.0, mode=Mode.SUBTRACT)

    # 4x M3 bolt through holes & top counterbores
    for bx in [bolt_x1, bolt_x2]:
        for sy in [-1, 1]:
            with Locations([(bx, sy * bolt_y, z_axis)]):
                Cylinder(radius=3.4 / 2, height=15.0, mode=Mode.SUBTRACT)
            with Locations([(bx, sy * bolt_y, z_axis + clamp_gap + ear_h - 2.5)]):
                Cylinder(radius=6.4 / 2, height=15.0, mode=Mode.SUBTRACT)

cap = cap_builder.part
print(f"   Top cap built! Volume: {cap.volume:.1f} mm^3")

# ====================================================================
# 5. EXPORT STEP & STL
# ====================================================================
out_dir = "/Users/manhngodh/development/robotics"
os.makedirs(out_dir, exist_ok=True)

print("4. Exporting STEP files...")
step_cradle_path = os.path.join(out_dir, "02_segment_dc180_cradle.step")
step_cap_path = os.path.join(out_dir, "02_segment_dc180_cap.step")
step_asm_path = os.path.join(out_dir, "02_segment_dc180_assembly.step")

export_step(full_segment_cradle, step_cradle_path)
export_step(cap, step_cap_path)

asm_compound = Compound([full_segment_cradle, cap])
export_step(asm_compound, step_asm_path)
print("   STEP files saved!")

print("5. Exporting STL files for 3D printing...")
stl_cradle_path = os.path.join(out_dir, "02_segment_dc180_cradle.stl")
stl_cap_path = os.path.join(out_dir, "02_segment_dc180_cap.stl")
export_stl(full_segment_cradle, stl_cradle_path)
export_stl(cap, stl_cap_path)

# Plate layout: Cradle flat at Z=0, Cap rotated 180 deg placed side-by-side
cap_print_loc = Location((46.5, 33.0, cap_top_z), (180, 0, 0))
cap_on_bed = cap.moved(cap_print_loc)
plate_compound = Compound([full_segment_cradle, cap_on_bed])
stl_plate_path = os.path.join(out_dir, "02_segment_dc180_plate.stl")
export_stl(plate_compound, stl_plate_path)
print("   STL files saved!")

# ====================================================================
# 6. PUSH TO ONSHAPE
# ====================================================================
print("6. Pushing to Onshape document 554c3f4204fffe9cc429ac54...")
try:
    with open(os.path.expanduser("~/.onshape/credentials.json")) as f:
        creds = json.load(f)
    client = Client(configuration={
        "base_url": creds.get("baseUrl", "https://cad.onshape.com"),
        "access_key": creds["accessKey"],
        "secret_key": creds["secretKey"]
    })

    did = "554c3f4204fffe9cc429ac54"
    wid = "a47520ea45b388137ec9a49b"

    with open(step_asm_path, "rb") as f:
        res = client.blob_elements_api.upload_file_create_element(
            did=did,
            wid=wid,
            file=f,
            translate=True,
            format_name="STEP",
            unit="millimeter",
            _preload_content=False
        )
    data = json.loads(res.data.decode("utf-8"))
    tid = data.get("translationId")
    print(f"   Upload OK! Translation ID: {tid}")

    if tid:
        print("   Waiting for translation to finish...")
        for _ in range(30):
            t_res = client.translation_api.get_translation(tid=tid, _preload_content=False)
            t_data = json.loads(t_res.data.decode("utf-8"))
            state = t_data.get("requestState")
            if state == "DONE":
                elems = t_data.get("resultElementIds", [])
                print(f"   Translation DONE! New Part Studio element: {elems}")
                new_eid = elems[0] if elems else ""
                print(f"   Direct URL: https://cad.onshape.com/documents/{did}/w/{wid}/e/{new_eid}")
                break
            elif state == "FAILED":
                print(f"   Translation failed: {t_data.get('failureReason')}")
                break
            time.sleep(1)

except Exception as ex:
    print(f"   Onshape push error: {ex}")

print("\nDone all tasks!")

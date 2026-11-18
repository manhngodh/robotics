#!/usr/bin/env python3
"""
Parametric CAD Generator for DC 180 Motor Mount Clamp
Generates STEP and STL models using build123d.
"""

from build123d import *

# ====================================================================
# PARAMETERS
# ====================================================================
# DC 180 Motor specifications
motor_w = 20.4        # Motor width (outer cylindrical diameter) [mm]
motor_t = 15.4        # Motor thickness (flat-to-flat distance) [mm]
motor_len = 32.0      # Motor metal body length [mm]
clearance = 0.25      # 3D printing fit tolerance (per side) [mm]

# Cavity dimensions with tolerance
w_c = motor_w + 2 * clearance    # 20.9 mm
t_c = motor_t + 2 * clearance    # 15.9 mm
r_c = w_c / 2                    # 10.45 mm
h_c = t_c / 2                    # 7.95 mm

# Clamp structural dimensions
clamp_len = 24.0      # Total length of clamp body [mm]
base_h_axis = 11.2    # Height of motor center axis from base bottom [mm]
body_w = 37.0         # Width of clamping ears block [mm]
flange_w = 51.0       # Total width of base plate [mm]
flange_h = 4.5        # Base plate thickness [mm]
flange_corner_r = 3.5 # Corner rounding radius [mm]
clamp_gap = 1.0       # Clamping gap between cap & base [mm]

# Fasteners (M3)
clamp_bolt_pitch = 29.0  # Distance between clamping bolts (X = +/- 14.5)
clamp_bolt_dia = 3.4     # M3 clearance hole
clamp_head_dia = 6.4     # M3 socket head cap screw counterbore
clamp_head_depth = 2.5   # Counterbore depth
nut_flat_w = 5.85        # M3 hex nut width across flats
nut_depth = 3.2          # M3 hex nut pocket depth

# Base mounting holes (4x M3)
mount_pitch_x = 43.0     # X pitch
mount_pitch_y = 14.0     # Y pitch
mount_hole_dia = 3.5     # M3 clearance
mount_head_dia = 6.5     # M3 counterbore
mount_head_depth = 2.2   # M3 counterbore depth

# Retention & Cooling
front_stop_thick = 2.0   # Front thrust retention lip thickness
front_aperture_dia = 10.0 # Front opening for collar and shaft
vent_slot_w = 7.0        # Vent slot width
vent_slot_l = 15.0       # Vent slot length

# Plane at motor center axis pointing along +Y
plane_motor = Plane(origin=(0, 0, base_h_axis), x_dir=(1, 0, 0), z_dir=(0, 1, 0))

# --------------------------------------------------------------------
# 1. BUILD BASE PART
# --------------------------------------------------------------------
with BuildPart() as base_builder:
    # A. Flange plate with rounded corners
    with BuildSketch() as s_flange:
        RectangleRounded(flange_w, clamp_len, radius=flange_corner_r)
    extrude(s_flange.sketch, amount=flange_h)
    
    # B. Main clamping body block
    with BuildSketch(Plane.XY.offset(flange_h)) as s_body:
        Rectangle(body_w, clamp_len)
    extrude(s_body.sketch, amount=base_h_axis - flange_h)
    
    # C. Motor cavity cut (leaving front stop lip)
    cavity_start = -clamp_len / 2 + front_stop_thick
    cavity_depth = clamp_len - front_stop_thick + 2.0
    with BuildSketch(plane_motor.offset(cavity_start)) as s_cavity:
        Circle(radius=r_c)
        Rectangle(r_c * 2 + 5, h_c * 2, mode=Mode.INTERSECT)
    extrude(s_cavity.sketch, amount=cavity_depth, mode=Mode.SUBTRACT)
    
    # D. Front aperture (10mm) for shaft/bearing collar
    with BuildSketch(plane_motor.offset(-clamp_len / 2 - 1.0)) as s_front:
        Circle(radius=front_aperture_dia / 2)
        # 2x M2 front screw clearance holes
        with Locations([(-5.0, 0), (5.0, 0)]):
            Circle(radius=2.4 / 2)
    extrude(s_front.sketch, amount=front_stop_thick + 2.0, mode=Mode.SUBTRACT)
    
    # E. Clamp screw holes & bottom hex nut traps
    bolt_x = clamp_bolt_pitch / 2
    for sx in [-1, 1]:
        # Through hole
        with Locations([(sx * bolt_x, 0, -1.0)]):
            Cylinder(radius=clamp_bolt_dia / 2, height=base_h_axis + 2.0, mode=Mode.SUBTRACT)
        # Hex nut pocket from bottom
        with Locations([(sx * bolt_x, 0, 0)]):
            with BuildSketch() as s_nut:
                RegularPolygon(radius=nut_flat_w / (3**0.5), side_count=6)
            extrude(s_nut.sketch, amount=nut_depth, mode=Mode.SUBTRACT)
            
    # F. Base mounting holes (4x M3 with counterbores)
    for sx in [-1, 1]:
        for sy in [-1, 1]:
            pos = (sx * mount_pitch_x / 2, sy * mount_pitch_y / 2)
            with Locations([(pos[0], pos[1], -1.0)]):
                Cylinder(radius=mount_hole_dia / 2, height=flange_h + 2.0, mode=Mode.SUBTRACT)
            with Locations([(pos[0], pos[1], flange_h - mount_head_depth)]):
                Cylinder(radius=mount_head_dia / 2, height=10.0, mode=Mode.SUBTRACT)
                
    # G. Bottom ventilation slot
    with BuildSketch(Plane.XY.offset(-1.0)) as s_vent:
        Rectangle(vent_slot_w, vent_slot_l)
    extrude(s_vent.sketch, amount=base_h_axis + 2.0, mode=Mode.SUBTRACT)

base_part = base_builder.part

# --------------------------------------------------------------------
# 2. BUILD TOP CAP PART
# --------------------------------------------------------------------
cap_top_z = base_h_axis + h_c + 3.2
ear_h = 5.4

with BuildPart() as cap_builder:
    # A. Cap outer solid hull
    # Base block at ears level
    with BuildSketch(Plane.XY.offset(base_h_axis + clamp_gap)) as s_ear:
        Rectangle(body_w, clamp_len)
    b_ear = extrude(s_ear.sketch, amount=ear_h)
    
    # Top arch crown block
    with BuildSketch(Plane.XY.offset(cap_top_z - 2.5)) as s_top:
        Rectangle(r_c * 1.5, clamp_len)
    b_top = extrude(s_top.sketch, amount=2.5)
    
    # Loft / Hull between ears and top crown
    loft([s_ear.sketch, s_top.sketch])
    
    # B. Motor cavity cut
    with BuildSketch(plane_motor.offset(cavity_start)) as s_cap_cavity:
        Circle(radius=r_c)
        Rectangle(r_c * 2 + 5, h_c * 2, mode=Mode.INTERSECT)
    extrude(s_cap_cavity.sketch, amount=cavity_depth, mode=Mode.SUBTRACT)
    
    # C. Front aperture cut
    with BuildSketch(plane_motor.offset(-clamp_len / 2 - 1.0)) as s_cap_front:
        Circle(radius=front_aperture_dia / 2)
    extrude(s_cap_front.sketch, amount=front_stop_thick + 2.0, mode=Mode.SUBTRACT)
    
    # D. Clamp screw through-holes & counterbores on top
    for sx in [-1, 1]:
        with Locations([(sx * bolt_x, 0, base_h_axis)]):
            Cylinder(radius=clamp_bolt_dia / 2, height=25.0, mode=Mode.SUBTRACT)
        with Locations([(sx * bolt_x, 0, base_h_axis + clamp_gap + ear_h - clamp_head_depth)]):
            Cylinder(radius=clamp_head_dia / 2, height=15.0, mode=Mode.SUBTRACT)
            
    # E. Top vent slot
    with BuildSketch(Plane.XY.offset(base_h_axis)) as s_top_vent:
        Rectangle(vent_slot_w, vent_slot_l)
    extrude(s_top_vent.sketch, amount=25.0, mode=Mode.SUBTRACT)

cap_part = cap_builder.part

# --------------------------------------------------------------------
# 3. EXPORT STEP & STL FILES
# --------------------------------------------------------------------
print("Exporting CAD STEP files...")
export_step(base_part, "dc180_mount_clamp_base.step")
export_step(cap_part, "dc180_mount_clamp_cap.step")

# Assembly compound
assembly = Compound([base_part, cap_part])
export_step(assembly, "dc180_mount_clamp_assembly.step")

print("Export completed successfully!")
print(f"Base volume: {base_part.volume:.2f} mm^3")
print(f"Cap volume:  {cap_part.volume:.2f} mm^3")

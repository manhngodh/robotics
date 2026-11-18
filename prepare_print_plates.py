import os
from build123d import *

out_dir = "/Users/manhngodh/development/robotics"

# 1. Load exported STLs
raw_u_break = import_stl(os.path.join(out_dir, "u_break.stl"))
raw_housing = import_stl(os.path.join(out_dir, "housing.stl"))
raw_plate = import_stl(os.path.join(out_dir, "plate.stl"))
raw_compound = import_stl(os.path.join(out_dir, "segment_horizontal_compound.stl"))
raw_part3 = import_stl(os.path.join(out_dir, "segment_horizontal_part3.stl"))

# --- Helper to center shape on XY and place on Z=0 ---
def place_on_bed(shape):
    bb = shape.bounding_box()
    # Translate so min Z is 0 and center is at (0, 0)
    dx = -(bb.min.X + bb.max.X) / 2.0
    dy = -(bb.min.Y + bb.max.Y) / 2.0
    dz = -bb.min.Z
    return shape.translate((dx, dy, dz))

# 2. Orient each part optimally for FDM 3D printing

# U-Break: already flat base at Z=0
p_ubreak = place_on_bed(raw_u_break)
export_stl(p_ubreak, os.path.join(out_dir, "PrintReady_UBreak_Bracket.stl"))

# Housing: bottom flat at Z=0
p_housing = place_on_bed(raw_housing)
export_stl(p_housing, os.path.join(out_dir, "PrintReady_DC180_Housing.stl"))

# Plate (DC180 Cap): Flip 180 degrees around X so flat clamping top rests on bed
flipped_plate = raw_plate.rotate(Axis.X, 180)
p_cap = place_on_bed(flipped_plate)
export_stl(p_cap, os.path.join(out_dir, "PrintReady_DC180_Cap.stl"))

# Segment Compound: flat base at Z=0
p_compound = place_on_bed(raw_compound)
export_stl(p_compound, os.path.join(out_dir, "PrintReady_HorizontalSegment_Body.stl"))

# Segment Part 3: Lay flat (rotate 90 deg around Y) so the 21x22 face is on bed
laid_part3 = raw_part3.rotate(Axis.Y, -90)
p_part3 = place_on_bed(laid_part3)
export_stl(p_part3, os.path.join(out_dir, "PrintReady_HorizontalSegment_Cap.stl"))

print("Individual print-ready STLs exported successfully!")

# 3. Create Full Plate Combinations

# Plate 1: DC180 Toolhead Set (Housing + Cap + UBreak)
# Lay out: U-Break on left, Housing in middle, Cap on right
plate1_parts = Compound([
    p_ubreak.translate((-45.0, 0.0, 0.0)),
    p_housing.translate((15.0, 0.0, 0.0)),
    p_cap.translate((15.0, 35.0, 0.0))
])
export_stl(plate1_parts, os.path.join(out_dir, "Plate1_DC180_Toolhead_FullSet.stl"))
print(f"Plate 1 exported: bbox={plate1_parts.bounding_box().size.X:.1f} x {plate1_parts.bounding_box().size.Y:.1f} mm")

# Plate 2: Horizontal Segment Set (Compound + Part 3 + UBreak)
plate2_parts = Compound([
    p_ubreak.translate((-40.0, 0.0, 0.0)),
    p_compound.translate((15.0, 0.0, 0.0)),
    p_part3.translate((15.0, 30.0, 0.0))
])
export_stl(plate2_parts, os.path.join(out_dir, "Plate2_HorizontalSegment_FullSet.stl"))
print(f"Plate 2 exported: bbox={plate2_parts.bounding_box().size.X:.1f} x {plate2_parts.bounding_box().size.Y:.1f} mm")

# Master Plate: All 5 parts arranged comfortably on SPARKX i7 bed (250x250)
master_parts = Compound([
    # Left column: U-Break
    p_ubreak.translate((-50.0, 0.0, 0.0)),
    # Middle column: DC180 Housing & Cap
    p_housing.translate((10.0, -25.0, 0.0)),
    p_cap.translate((10.0, 25.0, 0.0)),
    # Right column: Segment Body & Segment Cap
    p_compound.translate((65.0, -20.0, 0.0)),
    p_part3.translate((65.0, 25.0, 0.0))
])
export_stl(master_parts, os.path.join(out_dir, "MasterPlate_AllParts_ReadyToPrint.stl"))
print(f"Master Plate exported: bbox={master_parts.bounding_box().size.X:.1f} x {master_parts.bounding_box().size.Y:.1f} mm")

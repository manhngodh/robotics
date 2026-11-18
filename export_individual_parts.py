import os
import shutil
import subprocess
from build123d import import_step, import_stl, export_step, export_stl

base_dir = "/Users/manhngodh/development/robotics"
target_dir = os.path.join(base_dir, "individual_parts")
os.makedirs(target_dir, exist_ok=True)

# 1. Load assemblies STEP to extract exact CAD STEP solids
asm1 = import_step(os.path.join(base_dir, "DC180_UBreak_Assemble.step"))
solids1 = asm1.solids()
# Solid 0: housing, Solid 1: plate, Solid 2: u_break
cad_housing = solids1[0]
cad_plate = solids1[1]
cad_ubreak = solids1[2]

asm2 = import_step(os.path.join(base_dir, "Assembly_1.step"))
solids2 = asm2.solids()
# Solid 0: segment_compound, Solid 1: segment_part3, Solid 2: u_break
cad_compound = solids2[0]
cad_part3 = solids2[1]

# 2. Export individual STEP files
export_step(cad_ubreak, os.path.join(target_dir, "01_UBreak_Bracket.step"))
export_step(cad_housing, os.path.join(target_dir, "02_DC180_Toolhead_Housing.step"))
export_step(cad_plate, os.path.join(target_dir, "03_DC180_Toolhead_Cap.step"))
export_step(cad_compound, os.path.join(target_dir, "04_HorizontalSegment_Body.step"))
export_step(cad_part3, os.path.join(target_dir, "05_HorizontalSegment_Cap.step"))
print("STEP files exported.")

# 3. Copy / Generate Print-Ready STL files
stl_map = {
    "01_UBreak_Bracket.stl": "PrintReady_UBreak_Bracket.stl",
    "02_DC180_Toolhead_Housing.stl": "PrintReady_DC180_Housing.stl",
    "03_DC180_Toolhead_Cap.stl": "PrintReady_DC180_Cap.stl",
    "04_HorizontalSegment_Body.stl": "PrintReady_HorizontalSegment_Body.stl",
    "05_HorizontalSegment_Cap.stl": "PrintReady_HorizontalSegment_Cap.stl",
}

for dest, src in stl_map.items():
    src_path = os.path.join(base_dir, src)
    dest_path = os.path.join(target_dir, dest)
    shutil.copyfile(src_path, dest_path)
    print(f"Copied STL: {dest}")

# 4. Also copy to ~/Downloads for one-click access from user's Mac Finder
downloads_dir = os.path.expanduser("~/Downloads/Robotics_Individual_Parts")
os.makedirs(downloads_dir, exist_ok=True)
for fname in os.listdir(target_dir):
    if fname.endswith((".stl", ".step")):
        shutil.copyfile(os.path.join(target_dir, fname), os.path.join(downloads_dir, fname))

# 5. Create a convenient ZIP file
zip_base = os.path.join(base_dir, "Robotics_All_Individual_Parts")
shutil.make_archive(zip_base, 'zip', target_dir)
shutil.copyfile(f"{zip_base}.zip", os.path.expanduser("~/Downloads/Robotics_All_Individual_Parts.zip"))
print(f"Created ZIP: {zip_base}.zip and in ~/Downloads")

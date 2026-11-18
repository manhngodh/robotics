import subprocess
import os

out_dir = "/Users/manhngodh/development/robotics"
scad_path = os.path.join(out_dir, "preview_all.scad")

# Write an OpenSCAD file to render assemblies and parts
with open(scad_path, "w") as f:
    f.write("""
$fn = 60;

// Assembly 1: DC180 Toolhead
module dc180_assembly() {
    color([0.2, 0.6, 0.9, 0.9]) import("housing.stl");
    color([0.9, 0.4, 0.1, 0.9]) import("plate.stl");
    color([0.3, 0.8, 0.3, 0.9]) translate([-11.54, 0, 0]) import("u_break.stl");
}

// Assembly 2: Horizontal Arm Segment
module horizontal_assembly() {
    color([0.8, 0.3, 0.8, 0.9]) import("segment_horizontal_compound.stl");
    color([0.9, 0.8, 0.2, 0.9]) import("segment_horizontal_part3.stl");
    color([0.3, 0.8, 0.3, 0.9]) translate([-37.49, 0, 0]) import("u_break.stl");
}

// Full Plate Layout for 3D Printing (Optimally oriented on bed)
module print_plate_layout() {
    // 1. DC180 Housing (Flat on bed)
    translate([0, 40, 0]) color([0.2, 0.6, 0.9]) import("housing.stl");
    // 2. DC180 Cap (Flipped flat)
    translate([0, -10, 26]) rotate([180, 0, 0]) color([0.9, 0.4, 0.1]) import("plate.stl");
    // 3. U-Break Bracket (Flat base on bed)
    translate([-60, 40, 0]) color([0.3, 0.8, 0.3]) import("u_break.stl");
    // 4. Segment Compound
    translate([60, 40, 0]) color([0.8, 0.3, 0.8]) import("segment_horizontal_compound.stl");
    // 5. Segment Part 3 (laid flat)
    translate([60, -10, 4.5]) rotate([0, -90, 0]) color([0.9, 0.8, 0.2]) import("segment_horizontal_part3.stl");
}

// Render mode selector: define which view to render via command line
""")

# Render individual images
views = [
    ("dc180_assembly()", "preview_dc180_ubreak_assemble.png", "--camera=30,-100,50,30,0,10,700"),
    ("horizontal_assembly()", "preview_segment_horizontal_assemble.png", "--camera=0,-120,40,0,0,10,700"),
    ("print_plate_layout()", "preview_full_print_plate.png", "--camera=0,-150,150,0,10,10,900")
]

for call, img_name, cam in views:
    temp_scad = os.path.join(out_dir, f"temp_{img_name}.scad")
    img_out = os.path.join(out_dir, img_name)
    with open(temp_scad, "w") as f:
        f.write(f'include <preview_all.scad>;\n{call};\n')
    cmd = [
        "/Applications/OpenSCAD.app/Contents/MacOS/OpenSCAD",
        "-o", img_out,
        "--imgsize=1200,900",
        "--colorscheme=Tomorrow Night",
        cam,
        temp_scad
    ]
    print(f"Rendering {img_name}...")
    subprocess.run(cmd, check=True)
    if os.path.exists(temp_scad):
        os.remove(temp_scad)
    print(f"Rendered {img_out}")

print("All renders complete!")

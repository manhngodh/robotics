import subprocess
import os

out_dir = "/Users/manhngodh/development/robotics"
scad_path = os.path.join(out_dir, "render_plates.scad")

with open(scad_path, "w") as f:
    f.write("""
$fn = 60;

module plate_grid() {
    color([0.2, 0.2, 0.25]) {
        for (x = [-100:20:100]) {
            translate([x, 0, -0.5]) cube([0.5, 200, 0.2], center=true);
        }
        for (y = [-100:20:100]) {
            translate([0, y, -0.5]) cube([200, 0.5, 0.2], center=true);
        }
    }
}

module view_plate1() {
    plate_grid();
    color([0.2, 0.65, 0.9]) import("Plate1_DC180_Toolhead_FullSet.stl");
}

module view_plate2() {
    plate_grid();
    color([0.8, 0.35, 0.85]) import("Plate2_HorizontalSegment_FullSet.stl");
}

module view_master() {
    plate_grid();
    color([0.2, 0.75, 0.5]) import("MasterPlate_AllParts_ReadyToPrint.stl");
}
""")

renders = [
    ("view_plate1()", "render_plate1_dc180.png", "--camera=0,-120,90,0,10,10,650"),
    ("view_plate2()", "render_plate2_horizontal.png", "--camera=0,-120,90,0,10,10,650"),
    ("view_master()", "render_master_plate.png", "--camera=0,-160,110,10,0,10,800")
]

for call, fname, cam in renders:
    tmp = os.path.join(out_dir, "tmp.scad")
    out_img = os.path.join(out_dir, fname)
    with open(tmp, "w") as f:
        f.write(f'include <render_plates.scad>;\n{call};\n')
    cmd = [
        "/Applications/OpenSCAD.app/Contents/MacOS/OpenSCAD",
        "-o", out_img,
        "--imgsize=1200,900",
        "--colorscheme=Tomorrow Night",
        cam,
        tmp
    ]
    subprocess.run(cmd, check=True)
    if os.path.exists(tmp):
        os.remove(tmp)
    print(f"Rendered {out_img}")

print("Done rendering plate images!")

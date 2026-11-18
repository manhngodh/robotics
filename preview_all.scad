
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


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


include <threaded_cap_test_ring.scad>
difference() {
    thread_test_ring();
    translate([0, -50, -5]) cube([100, 100, 20]);
}


$fn = 96;

test_height             = 10.0;    // Chiều cao 10mm (chứa 4 vòng ren rõ nét)
thread_nominal_major    = 35.0;    // Ren ngoài: 35.0 mm
thread_nominal_minor    = 33.0;    // Ren trong: 33.0 mm
thread_clearance        = 0.35;    // Dung sai bù in 3D (+0.35 mm)
thread_pitch            = 2.5;     // Bước ren 2.5 mm
thread_entry_chamfer    = 1.0;     // Vát mép dẫn hướng 2 đầu
outer_diameter_peaks    = 70.0;
outer_diameter_valleys  = 64.0;
bottom_chamfer          = 1.0;

module scalloped_profile_2d(r_peak, r_valley, n_lobes=8, steps=180) {
    r_mid = (r_peak + r_valley) / 2;
    amp   = (r_peak - r_valley) / 2;
    polygon(points = [
        for (i = [0 : steps - 1])
            let (a = i * 360 / steps,
                 r = r_mid + amp * cos(n_lobes * a))
            [r * cos(a), r * sin(a)]
    ]);
}

// Deep continuous V-cutter that cuts 95% of the pitch, leaving sharp bold inward teeth
module deep_thread_cutter(r_maj, r_min, pitch, length, steps_per_turn=48) {
    turns = (length + pitch * 3) / pitch;
    total_steps = floor(turns * steps_per_turn);
    dz = pitch / steps_per_turn;
    da = 360 / steps_per_turn;
    
    // Wedge cutting out the space between teeth
    // V-shaped groove: base width is 0.92 * pitch at r_maj, tip at r_min-1
    points = [
        for (i = [0 : total_steps])
            let (a = i * da, z = i * dz, ca = cos(a), sa = sin(a))
            each [
                [(r_maj + 0.5) * ca, (r_maj + 0.5) * sa, z - pitch * 0.46],
                [(r_min - 0.5) * ca, (r_min - 0.5) * sa, z],
                [(r_maj + 0.5) * ca, (r_maj + 0.5) * sa, z + pitch * 0.46],
                [0, 0, z]
            ]
    ];
    
    faces = [
        for (i = [0 : total_steps - 1])
            let (b0 = i * 4, b1 = (i + 1) * 4)
            each [
                [b0 + 0, b1 + 0, b1 + 1],
                [b0 + 0, b1 + 1, b0 + 1],
                [b0 + 1, b1 + 1, b1 + 2],
                [b0 + 1, b1 + 2, b0 + 2],
                [b0 + 2, b1 + 2, b1 + 3],
                [b0 + 2, b1 + 3, b0 + 3],
                [b0 + 3, b1 + 3, b1 + 0],
                [b0 + 3, b1 + 0, b0 + 0]
            ],
        [0, 1, 2, 3],
        [total_steps * 4 + 3, total_steps * 4 + 2, total_steps * 4 + 1, total_steps * 4 + 0]
    ];
    
    polyhedron(points=points, faces=faces, convexity=10);
}

module thread_test_ring_v2() {
    r_peak = outer_diameter_peaks / 2;
    r_valley = outer_diameter_valleys / 2;
    r_maj = (thread_nominal_major + thread_clearance) / 2;
    r_min = (thread_nominal_minor + thread_clearance) / 2;
    
    difference() {
        linear_extrude(height = test_height) {
            scalloped_profile_2d(r_peak, r_valley);
        }
        
        // Bore hole at minor diameter
        translate([0, 0, -2])
            cylinder(r = r_min, h = test_height + 4);
            
        // Cut the deep continuous helical V-groove
        translate([0, 0, -thread_pitch])
            deep_thread_cutter(
                r_maj = r_maj,
                r_min = r_min,
                pitch = thread_pitch,
                length = test_height + thread_pitch * 2
            );
            
        // Top entrance chamfer
        translate([0, 0, test_height - thread_entry_chamfer])
            cylinder(
                r1 = r_min,
                r2 = r_maj,
                h  = thread_entry_chamfer + 0.1
            );
            
        // Bottom entrance chamfer
        translate([0, 0, -0.05])
            cylinder(
                r1 = r_maj,
                r2 = r_min,
                h  = thread_entry_chamfer + 0.1
            );
            
        // Bottom outer edge chamfer
        difference() {
            translate([0, 0, -1])
                cylinder(r = r_peak + 5, h = bottom_chamfer + 1);
            translate([0, 0, -0.1])
                cylinder(
                    r1 = r_peak - bottom_chamfer,
                    r2 = r_peak + 0.1,
                    h  = bottom_chamfer + 0.2
                );
        }
    }
}

thread_test_ring_v2();

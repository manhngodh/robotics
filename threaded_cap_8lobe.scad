
// ====================================================================
// PARAMETRIC 3D MODEL: REINFORCED THROUGH-HOLE 8-LOBED THREADED KNOB
// PHIÊN BẢN REN ĐẬM SẮC NÉT (BOLD HEAVY-DUTY THREADS)
// ====================================================================

through_hole            = true;    // Lỗ ren thông suốt
boss_wall_thickness     = 3.5;     // Độ dày thành ống ren (mm)

/* [Kích thước ren] */
thread_nominal_major    = 35.0;    // Ren ngoài: 35.0 mm
thread_nominal_minor    = 33.0;    // Ren trong: 33.0 mm
thread_clearance        = 0.35;    // Dung sai bù in 3D (+0.35 mm)
thread_pitch            = 2.5;     // Bước ren 2.5 mm (ren to, sắc nét, dễ in FDM)
thread_entry_chamfer    = 1.2;     // Vát mép dẫn hướng 2 đầu

/* [Kích thước thân ngoài] */
outer_diameter_peaks    = 70.0;    // Đỉnh 8 cánh lượn sóng: 70 mm
outer_diameter_valleys  = 64.0;    // Đáy rãnh lượn sóng: 64 mm
outer_skirt_height      = 18.0;    // Chiều cao vành ngoài: 18 mm
boss_height             = 20.0;    // Chiều cao cổ ren: 20 mm
bottom_chamfer          = 1.5;     // Vát mép đáy ngoài

$fn = 96;

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

// Deep continuous V-cutter
module deep_thread_cutter(r_maj, r_min, pitch, length, steps_per_turn=48) {
    turns = (length + pitch * 3) / pitch;
    total_steps = floor(turns * steps_per_turn);
    dz = pitch / steps_per_turn;
    da = 360 / steps_per_turn;
    
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

module threaded_knob_through_bold() {
    r_peak   = outer_diameter_peaks / 2;
    r_valley = outer_diameter_valleys / 2;
    r_boss   = (thread_nominal_major + 2 * boss_wall_thickness) / 2;
    
    r_maj    = (thread_nominal_major + thread_clearance) / 2;
    r_min    = (thread_nominal_minor + thread_clearance) / 2;
    
    difference() {
        union() {
            // Thân đặc ngoài 8 cánh
            linear_extrude(height = outer_skirt_height) {
                scalloped_profile_2d(r_peak, r_valley);
            }
            
            // Cổ ren nhô cao 20mm
            cylinder(r = r_boss, h = boss_height);
            
            // Vai côn chuyển tiếp 45 độ
            translate([0, 0, outer_skirt_height - 2.0])
                cylinder(r1 = r_boss + 3.0, r2 = r_boss, h = 2.0);
        }
        
        // Khoét lỗ khoan tâm
        translate([0, 0, -2])
            cylinder(r = r_min, h = boss_height + 4);
            
        // Cắt rãnh ren xoắn sâu đậm nét
        translate([0, 0, -thread_pitch])
            deep_thread_cutter(
                r_maj = r_maj,
                r_min = r_min,
                pitch = thread_pitch,
                length = boss_height + thread_pitch * 2
            );
            
        // Vát mép dẫn hướng miệng ren TRÊN
        translate([0, 0, boss_height - thread_entry_chamfer])
            cylinder(
                r1 = r_min,
                r2 = r_maj,
                h  = thread_entry_chamfer + 0.1
            );
            
        // Vát mép dẫn hướng miệng ren DƯỚI
        translate([0, 0, -0.05])
            cylinder(
                r1 = r_maj,
                r2 = r_min,
                h  = thread_entry_chamfer + 0.1
            );
            
        // Vát mép bo tròn góc đáy ngoài
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

threaded_knob_through_bold();

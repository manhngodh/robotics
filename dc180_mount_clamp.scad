// ====================================================================
// PARAMETRIC 3D MODEL: GÁ KẸP ĐỘNG CƠ DC 180 (DC 180 MOTOR MOUNT CLAMP)
// THIẾT KẾ CHUYÊN DỤNG CHO IN 3D FDM - HOÀN TOÀN KHÔNG CẦN SUPPORT
// HỖ TRỢ BẮT KHUNG VÀ HỐC ĐAI ỐC LỤC GIÁC M3 / REN NHIỆT (HEAT-SET)
// ====================================================================

$fn = 96;

/* [Chế độ hiển thị & Xuất file / Render Mode] */
render_mode         = "assembled"; // ["assembled": Lắp ráp hoàn chỉnh, "plate": Cả 2 chi tiết đặt phẳng in 3D, "base": Chỉ đế dưới, "cap": Chỉ nắp kẹp trên]

/* [Thông số động cơ DC 180] */
motor_width         = 20.4;    // Chiều rộng thân motor (đường kính ngoài cung tròn) [mm]
motor_thickness     = 15.4;    // Chiều dày thân motor (khoảng cách giữa 2 mặt vát phẳng) [mm]
motor_length        = 32.0;    // Chiều dài thân kim loại motor [mm]
clearance           = 0.25;    // Khe hở dung sai in 3D FDM (mỗi bên) [mm]

/* [Kích thước gá kẹp] */
clamp_len           = 24.0;    // Chiều dài khối kẹp ôm motor [mm]
base_h_axis         = 11.2;    // Chiều cao tâm trục motor so với đáy đế [mm]
body_w              = 37.0;    // Chiều rộng khối kẹp tai siết [mm]
flange_w            = 51.0;    // Chiều rộng toàn bộ đế (bao gồm tai bắt khung) [mm]
flange_h            = 4.5;     // Chiều dày tai bắt khung [mm]
flange_corner_r     = 3.5;     // Bán kính bo tròn góc tai bắt khung [mm]
clamp_gap           = 1.0;     // Khe hở siết giữa nắp và đế (đảm bảo lực ép motor) [mm]

/* [Ốc siết kẹp (Clamping Screws)] */
clamp_bolt_pitch    = 29.0;    // Khoảng cách giữa 2 ốc siết M3 (X = +/- 14.5) [mm]
clamp_bolt_dia      = 3.4;     // Đường kính lỗ ốc M3 [mm]
clamp_head_dia      = 6.4;     // Đường kính đầu ốc M3 (khoét chìm) [mm]
clamp_head_depth    = 2.5;     // Chiều sâu khoét chìm đầu ốc M3 [mm]
clamp_fastener      = "hex_nut"; // ["hex_nut": Hốc đai ốc lục giác M3 từ đáy, "heat_set": Lỗ cấy ren đồng nhiệt M3 từ trên]
nut_flat_w          = 5.85;    // Hốc đai ốc M3 (bề rộng đối diện cạnh + dung sai) [mm]
nut_depth           = 3.2;     // Chiều sâu hốc đai ốc M3 [mm]
heat_set_dia        = 4.2;     // Đường kính lỗ ren nhiệt M3 [mm]
heat_set_depth      = 4.5;     // Chiều sâu lỗ ren nhiệt M3 [mm]

/* [Lỗ bắt đế vào khung (Base Mounting)] */
mount_pitch_x       = 43.0;    // Khoảng cách lỗ bắt khung theo phương X [mm]
mount_pitch_y       = 14.0;    // Khoảng cách lỗ bắt khung theo phương Y [mm]
mount_hole_dia      = 3.5;     // Lỗ ốc M3 bắt khung [mm]
mount_head_dia      = 6.5;     // Lỗ khoét chìm đầu ốc M3 bắt khung [mm]
mount_head_depth    = 2.2;     // Chiều sâu khoét chìm đầu ốc bắt khung [mm]

/* [Tính năng phụ] */
front_stop_enabled  = true;    // Bật gờ chặn đầu trước chống trượt dọc trục
front_stop_thick    = 2.0;     // Chiều dày gờ chặn trước [mm]
front_aperture_dia  = 10.0;    // Đường kính lỗ mở phía trước (thoát cổ bạc đạn 6mm & trục) [mm]
front_m2_holes      = true;    // 2 lỗ ốc M2 bắt mặt trước motor
front_m2_pitch      = 10.0;    // Khoảng cách 2 lỗ ốc M2 mặt trước [mm]
vent_slot_enabled   = true;    // Rãnh thoát nhiệt cho motor
vent_slot_w         = 7.0;     // Bề rộng rãnh thoát nhiệt [mm]
vent_slot_l         = 15.0;    // Chiều dài rãnh thoát nhiệt [mm]

// --- TÍNH TOÁN KÍCH THƯỚC NỘI BỘ ---
w_c = motor_width + 2 * clearance;
t_c = motor_thickness + 2 * clearance;
r_c = w_c / 2;
h_c = t_c / 2;
bolt_x = clamp_bolt_pitch / 2;

// Module tiết diện 2D thân motor DC 180 (hình tròn vát 2 mặt phẳng)
module motor_cross_section_2d(r=r_c, h=h_c) {
    intersection() {
        circle(r=r);
        square([2 * r + 2, 2 * h], center=true);
    }
}

// Module khoang rỗng chứa motor
module motor_cavity_cut() {
    cavity_len = front_stop_enabled ? (clamp_len - front_stop_thick + 1) : (clamp_len + 4);
    cavity_start_y = front_stop_enabled ? (-clamp_len/2 + front_stop_thick) : (-clamp_len/2 - 2);
    
    // Khoang ôm thân motor
    translate([0, cavity_start_y, base_h_axis])
    rotate([-90, 0, 0])
    linear_extrude(cavity_len)
    motor_cross_section_2d();
    
    // Gờ chặn trước: khoét lỗ thoát cổ trục & ốc mặt M2
    if (front_stop_enabled) {
        // Lỗ thoát cổ 10mm
        translate([0, -clamp_len/2 - 2, base_h_axis])
        rotate([-90, 0, 0])
        cylinder(d=front_aperture_dia, h=front_stop_thick + 3);
        
        // 2 lỗ M2 mặt trước
        if (front_m2_holes) {
            for (sx = [-1, 1]) {
                translate([sx * front_m2_pitch / 2, -clamp_len/2 - 2, base_h_axis])
                rotate([-90, 0, 0])
                cylinder(d=2.4, h=front_stop_thick + 3);
            }
        }
    }
    
    // Vát mép dẫn hướng đuôi (Lead-in chamfer) giúp trượt motor vào êm ái
    translate([0, clamp_len/2, base_h_axis])
    rotate([-90, 0, 0])
    cylinder(r1=r_c - 0.2, r2=r_c + 1.2, h=1.5);
}

// Bo tròn 4 góc của đế chữ nhật 2D
module rounded_rect_2d(size, r) {
    x = size[0];
    y = size[1];
    hull() {
        translate([-x/2 + r, -y/2 + r]) circle(r=r);
        translate([ x/2 - r, -y/2 + r]) circle(r=r);
        translate([-x/2 + r,  y/2 - r]) circle(r=r);
        translate([ x/2 - r,  y/2 - r]) circle(r=r);
    }
}

// Mô hình motor ảo 3D để preview trực quan
module motor_dummy() {
    color([0.78, 0.78, 0.82, 0.85]) {
        // Thân kim loại motor
        rotate([-90, 0, 0])
        translate([0, 0, -motor_length / 2])
        linear_extrude(motor_length)
        intersection() {
            circle(d=motor_width);
            square([motor_width + 2, motor_thickness], center=true);
        }
        
        // Cổ trước (collar 6mm)
        rotate([-90, 0, 0])
        translate([0, 0, -motor_length / 2 - 1.8])
        cylinder(d=6.1, h=1.8);
        
        // Trục trước (shaft 2mm)
        color([0.95, 0.95, 0.95])
        rotate([-90, 0, 0])
        translate([0, 0, -motor_length / 2 - 10.0])
        cylinder(d=2.0, h=10.0);
        
        // Chân cực hàn phía sau
        color([0.9, 0.7, 0.2]) {
            translate([-5.5, motor_length / 2, -1.0]) cube([1.6, 4.5, 2.0]);
            translate([ 3.9, motor_length / 2, -1.0]) cube([1.6, 4.5, 2.0]);
        }
    }
}

// ==========================================
// 1. ĐẾ GÁ DƯỚI (BASE MOUNT BRACKET)
// ==========================================
module dc180_base() {
    difference() {
        union() {
            // Tai bắt khung (Flange)
            linear_extrude(flange_h)
                rounded_rect_2d([flange_w, clamp_len], flange_corner_r);
                
            // Thân gá chính với vát chuyển tiếp mượt mà
            hull() {
                // Đáy nối liền với tai bắt khung
                translate([-body_w/2, -clamp_len/2, 0])
                    cube([body_w, clamp_len, flange_h]);
                // Mặt trên của đế (mặt tiếp xúc split plane)
                translate([-body_w/2, -clamp_len/2, flange_h])
                    cube([body_w, clamp_len, base_h_axis - flange_h]);
            }
        }
        
        // Khoét khoang chứa motor
        motor_cavity_cut();
        
        // 2 Lỗ ốc siết kẹp M3
        for (sx = [-1, 1]) {
            translate([sx * bolt_x, 0, -1])
                cylinder(d=clamp_bolt_dia, h=base_h_axis + 2);
                
            if (clamp_fastener == "hex_nut") {
                // Hốc đai ốc lục giác M3 từ mặt đáy
                translate([sx * bolt_x, 0, -0.01])
                    cylinder(d=nut_flat_w / cos(30), h=nut_depth, $fn=6);
            } else if (clamp_fastener == "heat_set") {
                // Lỗ cấy ren đồng nhiệt M3 từ mặt trên
                translate([sx * bolt_x, 0, base_h_axis - heat_set_depth])
                    cylinder(d=heat_set_dia, h=heat_set_depth + 1);
            }
        }
        
        // 4 Lỗ bắt khung M3 (có khoét chìm đầu ốc)
        for (sx = [-1, 1], sy = [-1, 1]) {
            translate([sx * mount_pitch_x / 2, sy * mount_pitch_y / 2, -1]) {
                cylinder(d=mount_hole_dia, h=flange_h + 2);
                translate([0, 0, flange_h - mount_head_depth + 1])
                    cylinder(d=mount_head_dia, h=10);
            }
        }
        
        // Rãnh tản nhiệt mặt đáy
        if (vent_slot_enabled) {
            translate([-vent_slot_w/2, -vent_slot_l/2, -1])
                cube([vent_slot_w, vent_slot_l, base_h_axis]);
        }
    }
}

// ==========================================
// 2. NẮP KẸP TRÊN (TOP CLAMP CAP)
// ==========================================
module dc180_cap(in_place=true) {
    cap_top_z = base_h_axis + h_c + 3.2; // Độ cao đỉnh vòm
    ear_h     = 5.4;                    // Độ dày tai siết ốc
    
    difference() {
        // Thân nắp kẹp dạng vòm ôm khí động / tiết kiệm vật liệu
        hull() {
            // 2 tai kẹp 2 bên
            translate([-body_w/2, -clamp_len/2, base_h_axis + clamp_gap])
                cube([body_w, clamp_len, ear_h]);
                
            // Đỉnh vòm ôm sát motor
            translate([-r_c * 0.75, -clamp_len/2, cap_top_z - 2.5])
                cube([1.5 * r_c, clamp_len, 2.5]);
        }
        
        // Khoét khoang chứa motor
        motor_cavity_cut();
        
        // 2 Lỗ ốc siết M3 + khoét chìm đầu ốc M3 socket head
        for (sx = [-1, 1]) {
            translate([sx * bolt_x, 0, base_h_axis]) {
                cylinder(d=clamp_bolt_dia, h=25);
                translate([0, 0, clamp_gap + ear_h - clamp_head_depth])
                    cylinder(d=clamp_head_dia, h=15);
            }
        }
        
        // Rãnh tản nhiệt đỉnh
        if (vent_slot_enabled) {
            translate([-vent_slot_w/2, -vent_slot_l/2, base_h_axis])
                cube([vent_slot_w, vent_slot_l, 25]);
        }
    }
}

// ==========================================
// ĐIỀU KHIỂN XUẤT HÌNH (RENDER CONTROLLER)
// ==========================================
if (render_mode == "assembled") {
    // Chế độ mô phỏng lắp ráp hoàn chỉnh
    color([0.22, 0.45, 0.85]) dc180_base();
    translate([0, 0, 0]) motor_dummy();
    color([0.88, 0.38, 0.20]) dc180_cap(true);
    
    // Mô phỏng 2 ốc siết M3
    color([0.9, 0.9, 0.9]) {
        for (sx = [-1, 1]) {
            translate([sx * bolt_x, 0, base_h_axis + clamp_gap + 5.4 - 2.5])
                cylinder(d=5.5, h=3);
        }
    }
} else if (render_mode == "base") {
    // Chỉ xuất đế dưới
    dc180_base();
} else if (render_mode == "cap") {
    // Chỉ xuất nắp kẹp trên (đã lật ngửa 180 độ để in support-free)
    translate([0, 0, base_h_axis + h_c + 3.2])
    rotate([180, 0, 0])
    dc180_cap(true);
} else if (render_mode == "plate") {
    // Cả 2 chi tiết đặt cạnh nhau trên bàn in FDM (100% Support-free)
    translate([0, -17.0, 0]) dc180_base();
    translate([0,  17.0, base_h_axis + h_c + 3.2])
    rotate([180, 0, 0])
    dc180_cap(true);
}

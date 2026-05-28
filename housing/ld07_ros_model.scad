// LD07 LiDAR housing — parametric starter
// Dimensions from inno-maker / LDROBOT LD07 datasheet, page 6.
// Units: mm. Origin: center of PCB, laser exit along +Y, PCB plane = XY.

$fn = 64;

// --- LD07 module dimensions ---
pcb_w   = 25.60;   // PCB width  (X)
pcb_d   = 23.60;   // PCB depth  (Y)
pcb_t   = 1.60;    // PCB thickness (assumed; not in datasheet)
pcb_r   = 0.80;    // PCB corner radius

hole_dx = 22.10;   // mounting hole spacing X
hole_dy = 20.10;   // mounting hole spacing Y
hole_d  = 1.90;    // mounting hole diameter

opt_w   = 19.70;   // optical block width  (Y in datasheet "W")
opt_h_above = 6.01 + 6.70;  // total height of lens block above PCB
opt_front_offset = 1.20;    // lens face inset from front PCB edge
below_pcb = 7.17;  // component height below PCB

// ZH1.5T-4P connector mounted on TOP of PCB; cable exits straight up (+Z).
// Position is approximate — measure your unit and tweak.
conn_x  = 8.0;     // connector center X on PCB
conn_y  = -7.0;    // connector center Y on PCB
conn_w  = 5.0;     // connector body width  (X)
conn_l  = 7.0;     // connector body length (Y)
conn_h  = 4.0;     // connector body height above PCB
cable_d = 4.0;     // cable diameter for the top opening

fov_h = 90;        // horizontal field of view (deg)

// --- Housing parameters (tweak these) ---
wall      = 2.0;
floor_t   = 2.0;
clearance = 0.3;   // gap between PCB and housing wall
screw_pilot_d = 1.6;   // M2 self-tap pilot
post_d    = 4.5;
wing_extend = 5.0;   // -Y wall extends this far past the housing in ±X
wing_hole_d  = 3.0;  // mounting hole on each wing
wing_hole_dx = 31.0; // center-to-center spacing of wing holes

// =============================================================
// LD07 placeholder (for fit checks / visualization)
// =============================================================
module ld07_pcb() {
    color("darkgreen")
    linear_extrude(pcb_t)
        offset(r = pcb_r) offset(r = -pcb_r)
            square([pcb_w, pcb_d], center = true);

    // mounting holes (visualization only).
    // The (+X, −Y) corner is omitted — that hole is occupied by the connector.
    color("silver")
    for (pair = [[-1,-1], [-1, 1], [1, 1]])
        translate([pair[0] * hole_dx / 2, pair[1] * hole_dy / 2, -0.1])
            cylinder(d = hole_d, h = pcb_t + 0.2);
}

module ld07_optics() {
    // Optical block now on the UNDERSIDE of the PCB; laser exits −Z
    color("dimgray")
    translate([0, 0, -opt_h_above])
        linear_extrude(opt_h_above)
            translate([0, -opt_front_offset, 0])
                square([16, opt_w], center = true);
}

module ld07_underside() {
    color("dimgray")
    translate([0, 0, -below_pcb])
        linear_extrude(below_pcb)
            square([18, 14], center = true);
}

module ld07_module() {
    ld07_pcb();
    //ld07_optics();
    ld07_underside();
    // ZH connector body on top of the PCB, opening at the top (+Z)
    color("ivory")
    translate([conn_x - conn_w/2, conn_y - conn_l/2, pcb_t])
        cube([conn_w, conn_l, conn_h]);
}

// =============================================================
// Housing — simple cradle with screw posts and FOV opening
// =============================================================
module rounded_rect(w, d, r) {
    offset(r = r) offset(r = -r) square([w, d], center = true);
}

module fov_cutout_down(depth, thickness = 24) {
    // 90° fan opening downward (−Z). Apex is the laser source.
    half = fov_h / 2;
    x = tan(half) * depth;
    translate([0, -thickness/2, 0])
        rotate([-90, 0, 0])
            linear_extrude(thickness)
                polygon(points = [[0, 0], [x, depth], [-x, depth]]);
}

module housing() {
    // Inverted mount: housing is an upside-down cup. The PCB hangs from
    // standoffs under the top plate; screws insert from above. The bottom
    // is open so the optics block protrudes freely (−Z) out of the housing.
    top_h = conn_h + cable_d;   // space above PCB for connector + cable
    lip   = 1.5;                // wall extends this far below PCB top edge
                                // to capture the PCB

    outer_w = pcb_w + 2 * (wall + clearance);
    outer_d = pcb_d + 2 * (wall + clearance);
    total_h = lip + pcb_t + top_h + floor_t;

    difference() {
        // outer shell (open at the bottom) — with -Y wing extension
        translate([0, 0, -lip])
            linear_extrude(total_h)
                union() {
                    rounded_rect(outer_w, outer_d, 2);
                    // wing on -Y wall extending ±X by wing_extend
                    translate([0, -outer_d/2 + wall/2, 0])
                        square([outer_w + 2 * wing_extend, wall],
                               center = true);
                }

        // PCB + connector pocket (open downward)
        translate([0, 0, -lip - 0.01])
            linear_extrude(lip + pcb_t + top_h + 1)
                rounded_rect(pcb_w + 2 * clearance,
                             pcb_d + 2 * clearance, pcb_r);

        // cable opening through the top plate
        translate([conn_x, conn_y, pcb_t + top_h - 0.1])
            cylinder(d = max(cable_d, conn_w + 1),
                     h = floor_t + 0.2);

        // screw clearance holes through the standoffs only; top plate stays solid
        for (pair = [[-1,-1], [-1, 1], [1, 1]])
            translate([pair[0] * hole_dx / 2, pair[1] * hole_dy / 2,
                       pcb_t - 0.1])
                cylinder(d = screw_pilot_d, h = top_h);

        // mounting holes through the -Y wings, axis along Y
        for (sx = [-1, 1])
            translate([sx * wing_hole_dx / 2,
                       -outer_d/2 + wall/2,
                       -lip + total_h / 2])
                rotate([90, 0, 0])
                    cylinder(d = wing_hole_d, h = wall + 1, center = true);

        // remove -X and +X side walls (keep -Y wall, +Y wall, and top plate)
        for (sx = [-1, 1])
            translate([sx * (outer_w/2 - wall/2) - (wall + 1)/2,
                       -outer_d/2 + wall + 0.1,
                       -lip - 0.5])
                cube([wall + 1,
                      outer_d - 2 * wall - 0.2,
                      lip + pcb_t + top_h + 0.5]);
    }

    // standoffs hanging from the top plate down to the PCB
    for (pair = [[-1,-1], [-1, 1], [1, 1]])
        translate([pair[0] * hole_dx / 2, pair[1] * hole_dy / 2, pcb_t])
            difference() {
                cylinder(d = post_d, h = top_h);
                translate([0, 0, -0.1])
                    cylinder(d = screw_pilot_d, h = top_h + 0.2);
            }
}

// =============================================================
// Render
// =============================================================
// Rotate 90° CW around Y+ so laser-exit (+Y) becomes +Z in ROS sensor frame.
rotate([90, 0, -90]) {
    housing();
    ld07_module();      // ghosted sensor for visual check
}

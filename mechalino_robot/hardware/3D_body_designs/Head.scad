$fn = 100;
module head()
{
    N = 8;

    difference()
    {
        cylinder(5,d = 120,d = 120);

        translate([50,17,-1])cylinder(20,3/2,3/2);
        translate([50,-17,-1])cylinder(20,3/2,3/2);
        translate([-40,17,-1])cylinder(20,3/2,3/2);
        translate([-40,-17,-1])cylinder(20,3/2,3/2);
        
        translate([50,17,3])cylinder(15,7/2,7/2);
        translate([50,-17,3])cylinder(15,7/2,7.6/2);
        translate([-40,17,3])cylinder(15,7/2,7/2);
        translate([-40,-17,3])cylinder(15,7/2,7/2);
        translate([50,17,-1])cylinder(2,7/2,7/2);
        translate([50,-17,-1])cylinder(2,7/2,7.6/2);
        translate([-40,17,-1])cylinder(2,7/2,7/2);
        translate([-40,-17,-1])cylinder(2,7/2,7/2);

        
        for(i=[1:1:N]){
             rotate([0,0,(360/N)*i])
             translate([55,0,-1])cylinder(20,3/2,3/2);
             rotate([0,0,(360/N)*i])
             translate([-55,0,3])cylinder(15,7/2,7/2);
             rotate([0,0,(360/N)*i])
             translate([-55,0,-1])cylinder(2,7/2,7/2);
        }


    translate([0,0,4])linear_extrude(2)polygon([[35,35],[35,-35],[-25,-35],[-25,35]]);
    }
    rotate([0,0,90])translate([0,-40,5])linear_extrude(0.5)text("NES institute, AAU Klagenfurt",size = 3,halign = "center",valign = "center");
    rotate([0,0,90])translate([0,-45,5])linear_extrude(0.5)text("Marceau - Khalil",size = 2.5,halign = "center",valign = "center");
}

    


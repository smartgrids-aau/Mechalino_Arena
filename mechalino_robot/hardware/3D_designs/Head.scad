$fn = 100;
N = 4;

difference()
{
    union()
    {
        cylinder(2,d = 125);
        for(i=[1:1:N]){
            rotate([0,0,(360/N)*i])
                translate([-55,0,0])
                    cylinder(52,7,7);
        }
        rotate([0,0,45])
            difference()
            {
                translate([-10-1,-8-1,2-1])
                    cube([20+2,16+2,3.5+1+1]);
                translate([-10,-8-2,2-0.01])
                    cube([20,16+5,3.5]);
            }
    }
    for(i=[1:1:N]){
         rotate([0,0,(360/N)*i])
            translate([55,0,-1])
                cylinder(30,6,6);
    }
    for(i=[1:1:N]){
         rotate([0,0,(360/N)*i])
            translate([55,0,-1])
                cylinder(55,1.8,1.8);
    }
    difference()
    {
        for(i=[1:1:N]){
         rotate([0,0,(360/N)*i])
            translate([55,0,31])
                cylinder(22,9,9);
        }
        for(i=[1:1:N]){
             rotate([0,0,(360/N)*i])
                translate([55,0,31])
                    cylinder(22,7/2,7/2);
        }
    }
}





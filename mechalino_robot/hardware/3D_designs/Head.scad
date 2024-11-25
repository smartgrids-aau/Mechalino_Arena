$fn = 100;
N = 4;

difference()
{
    union()
    {
        cylinder(2,d = 120);
        for(i=[1:1:N]){
            rotate([0,0,(360/N)*i])
                translate([-55,0,0])
                    cylinder(22,7/2,7/2);
        }
    }
    for(i=[1:1:N]){
         rotate([0,0,(360/N)*i])
            translate([55,0,-1])
                cylinder(25,1.8,1.8);
    }
}

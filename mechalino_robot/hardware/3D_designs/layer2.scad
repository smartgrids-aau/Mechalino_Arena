$fn = 100;
height = 22.5;
d = 4;
inner_d = 1.8;
N = 2;
/*
difference()
{
    union()
    {
        for(i=[1:1:N]){
            rotate([0,0,(360/N)*i])
                translate([-20,0,0])
                    cylinder(height,d,d);
        }
    }
    for(i=[1:1:N]){
         rotate([0,0,(360/N)*i])
            translate([20,0,-1])
                cylinder(height+2,inner_d,inner_d);
    }
}*/

difference()
{
    cylinder(height,d,d);
    translate([0,0,-1])
        cylinder(height+2,inner_d,inner_d);
}


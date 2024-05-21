$fn = 100;
height = 25;
N = 4;

difference()
{
    union()
    {
        for(i=[1:1:N]){
            rotate([0,0,(360/N)*i])
                translate([-20,0,0])
                    cylinder(height,5,5);
        }
    }
    for(i=[1:1:N]){
         rotate([0,0,(360/N)*i])
            translate([20,0,-1])
                cylinder(height+2,1.8,1.8);
    }
}





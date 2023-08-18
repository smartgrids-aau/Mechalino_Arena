
module wheels()
{
    $fn = 1000;

    difference() 
    {
    cylinder(4.5+5,5,5);
        linear_extrude(height = 10, center = false, convexity = 1, $fn = 1000)
    circle(r = 1.5);
    N=25;
        translate([0,0,-1])
    for(i=[1:1:N]){
         rotate([0,0,(360/N)*i])
         translate([1.1,0]) linear_extrude(4.5)//translate([0.7,0])
              circle(2,$fn=3);
        
    }
    translate([0,0,5])cylinder(4.5+5,7/2,7/2);
    }
    translate([0,0,3.5])cylinder(10.5,1.5,1.5);
    translate([0,0,0])cylinder(4,1,1);
}


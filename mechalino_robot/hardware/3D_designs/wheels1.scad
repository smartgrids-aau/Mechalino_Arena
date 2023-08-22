/*module wheels()
{
    $fn = 250;

    difference() 
    {
        cylinder(5.5+5,5,5);
        linear_extrude(height = 10, center = false, convexity = 1, $fn = 1000)
            circle(r = 1.5);
        N=25;
        translate([0,0,-1])
            for(i=[1:1:N]){
                 rotate([0,0,(360/N)*i])
                    translate([1.1,0])
                        linear_extrude(4.5)
                            circle(2,$fn=3);
                
            }
        translate([0,0,5])cylinder(22+5,7/2,7/2);
    }
    translate([0,0,3.5])cylinder(10.5,1.5,1.5);
    translate([0,0,0])cylinder(4,1,1);
}

wheels();*/

N=25;
K = 5;
$fn = 100;
epsilon = 0.01;
big_cy = 6.85/2;
big_cy_h = 4.5;
servoHole = 1.1;
servoheadR = 3;
servohead = 3.5;
bottom_joint = 0.8;
bottom_wall = bottom_joint + servohead;
wall=3;
wheel_bounes=1.4;

union()
{
    difference()
    {
        union()
        {
            difference()
            {
                cylinder(bottom_wall+2*big_cy_h,big_cy+wall,big_cy+wall);
                translate([0,0,bottom_wall])
                    cylinder(2*big_cy_h+epsilon*2,big_cy,big_cy);
                for( i = [0:360/5:360])
                {
                    rotate([0,0,i])
                        translate([0,0,bottom_wall+big_cy_h+1])
                            cube([wheel_bounes,big_cy+wall+epsilon,big_cy_h+epsilon*2]);
                }
            };
        };
        
        translate([0,0,-epsilon])
            for(i=[1:1:N]){
                 rotate([0,0,(360/N)*i])
                    translate([1.1,0])
                        linear_extrude(servohead+epsilon*2)
                            circle(2,$fn=3);
            }
        translate([0,0,-epsilon])
            cylinder(big_cy_h+2*epsilon,1.55,1.55);
    };
    
    
}
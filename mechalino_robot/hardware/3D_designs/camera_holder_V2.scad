$fn = 100;
/*difference()
{
    cube([151+3+3.85+1+1+3.85+3,20,4]);

    translate([151+3+3.85+1+1+3.85/2,10,-1])
    {
        cylinder(h=6,d=3.85);
    }
    
    translate([3+3.85/2,10,-1])
    {
        cylinder(h=6,d=3.85);
    }
}*/
/*translate([3+3.85+1,10,2])
    cube([151,20,4]);*/
err = 0.2;
hs = 4.1+err;    
thk = 3;
union(){
    difference()
    {
        union()
        {
            translate([(151+3+hs+1+1+hs+3)/2+(41+thk*2+err)/2,10+(59+thk*2+err)/2,0])
            {
                rotate([0,0,90])
                {
                    rotate([0,0,90])
                    {
                        difference()
                        {
                            cube([151+3+hs+1+1+hs+3,20,4]);

                            translate([151+3+hs+1+1+hs/2,10,-1])
                            {
                                cylinder(h=6,d=hs);
                            }
                            
                            translate([3+hs/2,10,-1])
                            {
                                cylinder(h=6,d=hs);
                            }
                        }
                    }
                }
            }
            cube([41+thk*2+err,59+thk*2+err,25+thk+err]);
        }
        translate([thk,thk,thk])
            cube([41+err,59+err,25+thk+err+1]);
        translate([5.5,-1,thk+8])
            cube([41-5.5*2+thk*2+err,59+2+thk*2+err,25+thk*2+err]);
        translate([-1,5.5,thk+8])
            cube([41+2+thk*2+err,59-5.5*2+thk*2+err,25+thk*2+err]);
        translate([thk+3,thk+3,-1])
            cube([41-6+err,59-6,thk+2]);
    };
};    


        
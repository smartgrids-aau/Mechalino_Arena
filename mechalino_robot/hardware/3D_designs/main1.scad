$fn = 100;

module main()
{
    Y1 = 10;
    Y2 = 32-Y1;
    a=[[-8-9.5,Y1+Y2+8],[-8-9.5,Y1+Y2],[-9.5,Y1+Y2],[-9.5,Y2],[-8-9.5,Y2],[-8-9.5,0],[-9.5+41+8,0],[-9.5+41+8,Y2],[-9.5+41,Y2],[-9.5+41,Y1+Y2],[-9.5+41+8,Y1+Y2],[-9.5+41+8,Y1+Y2+8]];
            
        difference() 
        {
            translate([0,0,-9])cylinder(29,120/2,120/2);
            translate([0,7,-1])
            linear_extrude(31)
            polygon(a);
            mirror([0,1,0])
            translate([0,7,-1])
            linear_extrude(31)
            polygon(a);
            translate([-48.3/2+10,-10,5])rotate([90,0,0])    cylinder(50,2,2);
            translate([48.3/2+10,-10,5])rotate([90,0,0])    cylinder(50,2,2);
            translate([-48.3/2+10,-10,15])rotate([90,0,0])    cylinder(50,2,2);
            translate([48.3/2+10,-10,15])rotate([90,0,0])    cylinder(50,2,2);
            
            mirror([0,1,0])translate([-48.3/2+10,-10,5])rotate([90,0,0])    cylinder(50,2,2);
            mirror([0,1,0])translate([48.3/2+10,-10,5])rotate([90,0,0])    cylinder(50,2,2);
            mirror([0,1,0])translate([-48.3/2+10,-10,15])rotate([90,0,0])    cylinder(50,2,2);
            mirror([0,1,0])translate([48.3/2+10,-10,15])rotate([90,0,0])    cylinder(50,2,2);
            translate([10,0,-15])linear_extrude(40)
            polygon([[30,40],[30,60],[-40,60],[-40,40]]);
            translate([10,0,-15])linear_extrude(40)
            polygon([[30,-40],[30,-60],[-40,-60],[-40,-40]]);
            
    translate([55,0,-9])sphere(r = 4);
    translate([-55,0,-9])sphere(r = 4);
        //    cube(center = true,size = [10,15,20] );
        }
//    translate([40,0,-9])cylinder(9,4,4);
//    translate([40,0,-9])sphere(r = 4);
//    translate([-40,0,-9])cylinder(9,4,4);
//    translate([-40,0,-9])sphere(r = 4);
     
    difference()
    {
        translate([50,17,20])cylinder(15,7.5/2,7.5/2);
        translate([50,17,20])cylinder(116,3/2,3/2);
    }
    difference()
    {
        translate([50,-17,20])cylinder(15,7.5/2,7.5/2);
        translate([50,-17,20])cylinder(16,3/2,3/2);
    }
    difference()
    {
        translate([-40,17,20])cylinder(15,7.5/2,7.5/2);
        translate([-40,17,20])cylinder(16,3/2,3/2);
    }
    difference()
    {
        translate([-40,-17,20])cylinder(15,7.5/2,7.5/2);
        translate([-40,-17,20])cylinder(16,3/2,3/2);
    }
        
}
    


module servo()
{
    X=48.3;
    a=[[0,0],[40.5,0],[40.5,29.5],[7+40.5,29.5],[7+40.5,29.5+2.5],[40.5,29.5+2.5],[40.5,39.4],[0,39.4],[0,29.5+2.5],[-7,29.5+2.5],[-7,29.5],[0,29.5]];


    difference() 
    {
        linear_extrude(20.2)polygon(a);
        translate([-7/2,50,5])rotate([90,0,0])cylinder(50,2,2);
        translate([-7/2,50,15])rotate([90,0,0])cylinder(50,2,2);
        translate([-7/2+X,50,5])rotate([90,0,0])cylinder(50,2,2);
        translate([-7/2+X,50,15])rotate([90,0,0])cylinder(50,2,2);
        
    }
    translate([9.5,29.5+19,10])rotate([90,0,0])cylinder(19.5,5.8/2,5.8/2);
}
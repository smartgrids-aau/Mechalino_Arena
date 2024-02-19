$fn=30;
H=10;
R=3;
Wood = 12.3;
Rws = R + Wood;
e = 0.1;
inner = Rws + e;
difference()
{
    difference()
    {
        cylinder(h=H,r=Rws);
        translate([-Wood/2,0,-e])
            cube([Wood,inner,H+2*e]);
    }
}
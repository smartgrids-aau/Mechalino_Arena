use <main1.scad>
use <servo.scad>
use <wheels1.scad>
use <Head.scad>

color("MediumBlue")main();


translate([-10,10.1,0]) color("red") servo();
mirror([0,1,0])translate([-9,10.1,0]) color("red") servo();
translate([0,55,10])rotate([-90,0,0])color("Gray")wheels();
mirror([0,1,0])translate([0,55,10])rotate([-90,0,0])color("Gray")wheels();
translate([0,0,35])head();

translate([0,60,10])rotate([-90,0,0])cylinder(19,47/2,47/2);
mirror([0,1,0])translate([0,60,10])rotate([-90,0,0])cylinder(19,47/2,47/2);




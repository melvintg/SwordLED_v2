$fn = 100;

// Variable declarations
dBase = 25;
hBase = 5;
dInnerBase = 19.4; // 25 - 2.8x2 thickness

dSW = 15.5;
dCapSW = 17;
hCapSW = 1.6;
hSW = 13;
hPinSW = 20;

hBat = 67;
dBat = 18.6;
dPinBat = 10;
hPinBat = 10;
hGapBat = 2;

hPcbGap = 8;
pins2pcbGap = 8;
pinsHole = dInnerBase - pins2pcbGap*2;
hole2pcbGap = 4;
M3HoleCenter = (dInnerBase/2) - hole2pcbGap;
dM3Hole = 3.1;

hTotal = hPinSW+hBat*2+hGapBat*2+hPinBat+hPcbGap;


module Slot() {
    difference(){
    cylinder(h=hTotal,d=dInnerBase);
    translate([-dInnerBase/2,-dInnerBase,hSW])
        cube([dInnerBase,dInnerBase,hTotal]);
    }
}


module SW() {
    union(){
        cylinder(h=hPinSW,d=dSW);
        cylinder(h=hCapSW,d=dCapSW);
    }
}

module Base() {
    cylinder(h=hBase,d=dBase);
}

module Batteries() {
    translate([0,0,hPinSW+hGapBat])
        cylinder(h=hBat,d=dBat);
    translate([0,0,hPinSW+hGapBat*2+hBat])
        cylinder(h=hBat,d=dBat);
    translate([0,0,hPinSW+hGapBat*2+hBat*2])
        cylinder(h=hPinBat,d=dPinBat);
    
}

module PCB(){
    translate([-M3HoleCenter,dInnerBase/2,hTotal-hole2pcbGap])
        rotate([90,90,0])
            cylinder(h=dInnerBase,d=dM3Hole);
    translate([M3HoleCenter,dInnerBase/2,hTotal-hole2pcbGap])
        rotate([90,90,0])
            cylinder(h=dInnerBase,d=dM3Hole);
    translate([0,0,hTotal-hPcbGap/2])
        cube([pinsHole,2,hPcbGap],true);
    
}

difference(){
    union(){
        Slot();
        Base();
    }
    SW();
    Batteries();
    PCB();
    /*
    translate([-dBase/2,-dInnerBase,0])
        cube([dBase,dInnerBase,hTotal]);
    */
}




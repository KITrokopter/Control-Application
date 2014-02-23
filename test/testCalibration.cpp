#include "Vector.h"
#include "Line.h"
#include "engine.h"
#include "AmccCalibration.h"
#include "Matlab.h"
#include "TrackingArea.h"
#include "profiling.hpp"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <math.h>

void calibrate(Matlab *m) {
    AmccCalibration *h = new AmccCalibration(m->getEngine());
    h->multiCameraCalibration(2, 30, 30, 11, 8);
}

void tracking(Matlab *m) {
    Vector *c1 = new Vector(0, 0, 0);
    Vector *c2 = new Vector(0, 10, 0);
    Vector *c3 = new Vector(10, 10, 0);
    Vector *c4 = new Vector(10, 0, 0);
    Vector *d1 = new Vector(10, 10, 10);
    Vector *d2 = new Vector(10, -10, 10);
    Vector *d3 = new Vector(-10, -10, 10);
    Vector *d4 = new Vector(-10, 10, 10);
    Vector cameraPosition[4] = {*c1, *c2, *c3, *c4};
    Vector cameraDirection[4] = {*d1, *d2, *d3, *d4};
    t = new TrackingArea(cameraPosition, cameraDirection, 4, 15, m->getEngine());
    printf("[%f %f %f], [%f %f %f], [%f %f %f], [%f, %f, %f], [%f, %f, %f] is the tracking area\n", t->getA1().getV1(), t->getA1().getV2(),t->getA1().getV3(), t->getA2().getV1(), t->getA2().getV2(), t->getA2().getV3(), t->getA3().getV1(), t->getA3().getV2(), t->getA3().getV3(), t->getA4().getV1(), t->getA4().getV2(), t->getA4().getV3(), t->getB1().getV1(), t->getB1().getV2(), t->getB1().getV3());

}

int main(int argc, char** argv) {
    Matlab *m = new Matlab();
    calibrate(m);
}

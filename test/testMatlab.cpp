#include "../src/matlab/Vector.h"
#include "../src/matlab/Line.h"
#include "../src/matlab/engine.h"
#include "../src/matlab/AmccCalibration.h"
#include "../src/matlab/Matlab.h"
#include "../src/matlab/TrackingArea.h"
#include "../src/matlab/profiling.hpp"
#include "../src/matlab/Position.h"
#include "../src/position/ChessboardData.hpp"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <math.h>

bool calibrate(Matlab *m) {
    Position *h = new Position(m->getEngine(), 3);
    ChessboardData *c = new ChessboardData(7, 7, 57, 57);
    //bool ok = h->calibrate(c, 3);
    /*Vector cam0 = h->getPositionInCameraCoordination(0);
    printf("[%f, %f, %f]\n", cam0.getV1(), cam0.getV2(), cam0.getV3());
    Vector cam1 = h->getPositionInCameraCoordination(1);
    printf("[%f, %f, %f]\n", cam1.getV1(), cam1.getV2(), cam1.getV3());
    cam0 = h->getPosition(0);
    printf("[%f, %f, %f]\n", cam0.getV1(), cam0.getV2(), cam0.getV3());*/
    h->setNumberCameras(3);
    Vector cam1 = h->getPosition(1);
    printf("cam 1 is [%f, %f, %f]\n", cam1.getV1(), cam1.getV2(), cam1.getV3());
    Vector cam2 = h->getPosition(2);
    printf("cam 2 is [%f, %f, %f]\n", cam2.getV1(), cam2.getV2(), cam2.getV3());
    Vector x = *(new Vector(1, 2, 3));
    h->updatePosition(x, 0, 1);
    x = *(new Vector(2, 3, 4));
    h->updatePosition(x, 1, 1);
    x = *(new Vector(1, 3, 2));
    h->updatePosition(x, 2, 1);
    Vector y = *(new Vector(1, 2, 4));
    Vector movement = h->updatePosition(y, 0, 1);
    y = *(new Vector(1, 2, 5));
    movement = h->updatePosition(y, 0, 1);
    printf("Quadcopter 1 moved in direction [%f, %f, %f]\n", movement.getV1(), movement.getV2(), movement.getV3());
    return true;
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
    TrackingArea *t = new TrackingArea(cameraPosition, cameraDirection, 4, 15, m->getEngine());
    printf("[%f %f %f], [%f %f %f], [%f %f %f], [%f, %f, %f], [%f, %f, %f] is the tracking area\n", t->getA1().getV1(), t->getA1().getV2(),t->getA1().getV3(), t->getA2().getV1(), t->getA2().getV2(), t->getA2().getV3(), t->getA3().getV1(), t->getA3().getV2(), t->getA3().getV3(), t->getA4().getV1(), t->getA4().getV2(), t->getA4().getV3(), t->getB1().getV1(), t->getB1().getV2(), t->getB1().getV3());

}

int main(int argc, char** argv) {
    Matlab *m = new Matlab();
    printf("Hallo das ist ein Test\n");
    //AmccCalibration *a= new AmccCalibration(m->getEngine());
    Engine *e = m->getEngine();
    Position *p = new Position(e,3);

    bool ok = calibrate(m);
    if (ok == true) {
        printf("success\n");
    } else {
        printf("fail\n");
    }
    m->destroyMatlab();
}

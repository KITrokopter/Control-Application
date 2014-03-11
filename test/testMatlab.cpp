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
    bool ok = h->calibrate(c, 3);
    Vector cam0 = h->getPosition(0);
    printf("cam 0 is [%f, %f, %f]\n", cam0.getV1(), cam0.getV2(), cam0.getV3());
    Vector cam1 = h->getPosition(1);
    printf("cam 1 is [%f, %f, %f]\n", cam1.getV1(), cam1.getV2(), cam1.getV3());
    Vector cam2 = h->getPosition(2);
    printf("cam 2 is [%f, %f, %f]\n", cam2.getV1(), cam2.getV2(), cam2.getV3());
    Vector x = *(new Vector(0.000002,0.0001, 0.99));
    h->updatePosition(x, 0, 1);
    h->updatePosition(x, 1, 1);
    Vector pos = h->updatePosition(x, 2, 1);
    printf("Quadcopter 1 moved at position [%f, %f, %f]\n", pos.getV1(), pos.getV2(), pos.getV3());
    pos = h->updatePosition(x, 2, 1);
       printf("Quadcopter 1 moved at position [%f, %f, %f]\n", pos.getV1(), pos.getV2(), pos.getV3());
       pos = h->updatePosition(x, 2, 1);
          printf("Quadcopter 1 moved at position [%f, %f, %f]\n", pos.getV1(), pos.getV2(), pos.getV3());
          pos = h->updatePosition(x, 2, 1);
             printf("Quadcopter 1 moved at position [%f, %f, %f]\n", pos.getV1(), pos.getV2(), pos.getV3());
             pos = h->updatePosition(x, 2, 1);
                printf("Quadcopter 1 moved at position [%f, %f, %f]\n", pos.getV1(), pos.getV2(), pos.getV3());
                pos = h->updatePosition(x, 2, 1);
                   printf("Quadcopter 1 moved at position [%f, %f, %f]\n", pos.getV1(), pos.getV2(), pos.getV3());
    return ok;
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
    //TrackingArea *t = new TrackingArea(cameraPosition, cameraDirection, 4, 15, m->getEngine());
    //printf("[%f %f %f], [%f %f %f], [%f %f %f], [%f, %f, %f], [%f, %f, %f] is the tracking area\n", t->getA1().getV1(), t->getA1().getV2(),t->getA1().getV3(), t->getA2().getV1(), t->getA2().getV2(), t->getA2().getV3(), t->getA3().getV1(), t->getA3().getV2(), t->getA3().getV3(), t->getA4().getV1(), t->getA4().getV2(), t->getA4().getV3(), t->getB1().getV1(), t->getB1().getV2(), t->getB1().getV3());

}

void perp(Matlab *m) {

    //Lotfußpunkt f: (-7,5,3) Lotfußpunkt g: (-1, 1, 5)
    Vector* a = new Vector(-7, 2, -3);
    Vector* u = new Vector(0, 1, 2);
    Vector* b = new Vector(-3, -3, 3);
    Vector* v = new Vector(1, 2, 1);

    Line* f = new Line(*a, *u);
    Line* g = new Line(*b, *v);
    long int first = getNanoTime();
    Vector oneLine = m->perpFootOneLine(*f, *b);
    long int second = getNanoTime();
    printf("result is [%f, %f, %f] in time %ld\n", oneLine.getV1(), oneLine.getV2(), oneLine.getV3(), second-first);

    Vector* result[2];
    first = getNanoTime();
    int intersects = m->perpFootTwoLines(*f, *g, result);
    second = getNanoTime();
    if (intersects == 0) {
        printf("parallel or same\n");
    } else {
        if (intersects == 1) {
                printf("intersectionpoint is [%f, %f, %f] in time %ld\n", result[0]->getV1(), result[0]->getV2(), result[0]->getV3(), second-first);
        } else {
            if (intersects == 2) {
                printf("Lotfußpunkt von f ist [%f, %f, %f] int time %ld\n", result[0]->getV1(), result[0]->getV2(), result[0]->getV3(), second-first);
                printf("Lotfußpunkt von g ist [%f, %f, %f]\n", result[1]->getV1(), result[1]->getV2(), result[1]->getV3());
            }
        }
    }
}

int main(int argc, char** argv) {
    Matlab *m = new Matlab();
    /*Vector a1 = *(new Vector(0.000000, -0.000000, 0.000000));
    Vector u1 = * (new Vector(0.315551, -0.962110, -0.234050));
    Vector a2 = * (new Vector(730.285416, -753.080314, 0.000243));
    Vector u2 = *(new Vector(-1.187641, 1.253347, 0.301279));
    Vector a3 = *(new Vector(-765.780098, -595.685729, 0.000209));
    Vector u3 = *(new Vector(1.249163, 1.125928, -0.282776));
    Line *quadPositions = new Line[3];
    quadPositions[0] = Line(a1, u1);
    quadPositions[1] = Line(a2, u2);
    quadPositions[2] = Line(a3, u3);
    Vector re = m->interpolateLines(quadPositions, 3);
    printf("result is %f, %f, %f\n", re.getV1(), re.getV2(), re.getV3());*/
    //perp(m);
    bool ok = calibrate(m);
    if (ok == true) {
        printf("success\n");
    } else {
        printf("fail\n");
    }

    /*
    Vector a = Vector(0,0,0);
    Vector b = Vector(730.285416, -753.080314, 0.000243);
    Vector u = Vector (0.000010, -1.000000, 0.000010);
    Vector v = Vector(-1.000000, -0.500000, 0.000010);
    Line l1 = Line(a, u);
    Line l2 = Line(b, v);
    Vector *result[2];
    m->perpFootTwoLines(l1, l2, result);
    printf("skew: [%f, %f, %f]\n", result[0]->getV1(), result[0]->getV2(), result[0]->getV3());
    printf("skew: [%f, %f, %f]\n", result[1]->getV1(), result[1]->getV2(), result[1]->getV3());
*/
    m->destroyMatlab();
}

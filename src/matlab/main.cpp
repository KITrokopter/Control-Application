#include "Vector.h"
#include "Line.h"
#include "engine.h"
#include "Calibration.h"
#include "Matlab.h"
#include "TrackingArea.h"
#include "profiling.hpp"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <math.h>


int main(int argc, char** argv)
{
    Matlab *m = new Matlab();
    Calibration *h = new Calibration(m->getEngine());
    h->multiCameraCalibration(2, 30, 30, 11, 8);

    /*//Lotfußpunkt (6, 3, 1)
    Vector* a = new Vector(-2, 1, 7);
    Vector* u = new Vector(4, 1, -3);
    Vector* b = new Vector(10, 5, 7);

    //Lotfußpunkt f: (-7,5,3) Lotfußpunkt g: (-1, 1, 5)
    Vector* a = new Vector(-7, 2, -3);
    Vector* u = new Vector(0, 1, 2);
    Vector* b = new Vector(-3, -3, 3);
    Vector* v = new Vector(1, 2, 1);

    intersection point (3, 6, 11)
    Vector* a = new Vector(4, 2, 8);
    Vector* u = new Vector(-1, 4, 3);
    Vector* b = new Vector(5, 8, 21);
    Vector* v = new Vector(1, 1, 5);
*/
    //Lotfußpunkt f: (8, -1, 7) Lotfußpunkt g: (8, 11, 1)
    Vector* a = new Vector(3, -1, 7);
    Vector* u = new Vector(1, 0, 0);
    Vector* b = new Vector(2, 8, -5);
    Vector* v = new Vector(2, 1, 2);

    Line* f = new Line(a, u);
    Line* g = new Line(b, v);
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
    Vector *c1 = new Vector(0, 0, 0);
    Vector *c2 = new Vector(0, 1, 0);
    Vector *c3 = new Vector(1, 0, 0);
    Vector *c4 = new Vector(1, 1, 0);
    Vector *d1 = new Vector(1, 1, 1);
    Vector *d2 = new Vector(1, -1, 1);
    Vector *d3 = new Vector(-1, 1, 1);
    Vector *d4 = new Vector(-1, -1, 0);
    Vector cameraPosition[4] = {*c1, *c2, *c3, *c4};
    Vector cameraDirection[4] = {*d1, *d2, *d3, *d4};
    TrackingArea *t = new TrackingArea(cameraPosition, cameraDirection, 4, 1, m->getEngine());
    printf("[%f %f %f], [%f %f %f], [%f %f %f], [%f, %f, %f], [%f, %f, %f] is the tracking area\n", t->getA1().getV1(), t->getA1().getV2(),t->getA1().getV3(), t->getA2().getV1(), t->getA2().getV2(), t->getA2().getV3(), t->getA3().getV1(), t->getA3().getV2(), t->getA3().getV3(), t->getA4().getV1(), t->getA4().getV2(), t->getA4().getV3(), t->getB1().getV1(), t->getB1().getV2(), t->getB1().getV3());
    Vector *x = new Vector(0, 0, 20);
    a = new Vector(1, 1, 1);
    u = new Vector(0, 1, 0);
    v = new Vector(1, 0, 0);
    double dist = t->getDistPointPlane(*a, *u, *v, *x);
    printf("distance is %f\n", dist);
    m->destroyMatlab();
    return EXIT_SUCCESS;
}

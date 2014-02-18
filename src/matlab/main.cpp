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

    //Calibration *h = new Calibration(m->getEngine());
    //h->multiCameraCalibration(2, 30, 30, 11, 8);

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

/*
    //Lotfußpunkt f: (8, -1, 7) Lotfußpunkt g: (8, 11, 1)
    Vector* a = new Vector(3, -1, 7);
    Vector* u = new Vector(1, 0, 0);
    Vector* b = new Vector(2, 8, -5);
    Vector* v = new Vector(2, 1, 2);

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


    Line *f = new Line(*(new Vector(2, 2, 2)), *(new Vector(1, -1, 5)));
    Vector *f1 = new Vector(3, 2, 4);
    Line *g = new Line(*(new Vector(2, 1, 5)), *(new Vector(0, -1, 3)));
    Vector *g1 = new Vector(2, 2, 11);
    Line u = m->getIntersectionLine(*f, *f1, *g, *g1);
    printf("a is [%f, %f, %f], should be [2, 2, 2], u is [%f, %f, %f], should be [0, -1, 3]\n", u.getA().getV1(), u.getA().getV2(), u.getA().getV3(),u.getU().getV1(), u.getU().getV2(), u.getU().getV3());
*/
    Vector *a1 = new Vector(0, 0, 0);
    Vector *a2 = new Vector(0, 1, 0);
    Vector *a3 = new Vector(1, 1, 0);
    Vector *a4 = new Vector(1, 0, 0);
    Vector *b1 = new Vector(1, 1, 1);
    Vector *b2 = new Vector(1, -1, 1);
    Vector *b3 = new Vector(-1, -1, 1);
    Vector *b4 = new Vector(-1, 1, 1);
    Vector *c1 = new Vector(0, 0, 0);
    Vector *c2 = new Vector(0, 10, 0);
    Vector *c3 = new Vector(10, 10, 0);
    Vector *c4 = new Vector(10, 0, 0);
    Vector *d1 = new Vector(10, 10, 10);
    Vector *d2 = new Vector(10, -10, 10);
    Vector *d3 = new Vector(-10, -10, 10);
    Vector *d4 = new Vector(-10, 10, 10);
    TrackingArea *t = new TrackingArea(*a1, *a2, *a3, *a4, *b1, *b2, *b3, *b4);
    Vector *bsp1 = new Vector(-4, 1, 1);
    Vector *bsp2 = new Vector(6, 0, -2);
    Vector *bsp3 = new Vector(5, -1, -1);
    Vector *bsp4 = new Vector(-2, 5, 7);
    //Vector perp = t->getPerpPointPlane(*bsp1, *bsp2, *bsp3, *bsp4);
    //printf("[%f, %f, %f], should be (-4, 1, 1)", perp.getV1(), perp.getV2(), perp.getV3());
    //if (t->inTrackingArea(*c1, *d1, 10, *(new Vector(0.5, 0.5, 0.5)), m->getEngine())) printf("true");
    //if (t->contains(t->getCenter()) == false) printf("Not in tracking area.\n");
    //if (t->inTrackingArea(*a1, *b1, 1, *(new Vector(0.5, 0.5, 0.5)), m->getEngine())) printf("true\n");

    Vector cameraPosition[4] = {*c1, *c2, *c3, *c4};
    Vector cameraDirection[4] = {*d1, *d2, *d3, *d4};
    t = new TrackingArea(cameraPosition, cameraDirection, 4, 15, m->getEngine());
    printf("[%f %f %f], [%f %f %f], [%f %f %f], [%f, %f, %f], [%f, %f, %f] is the tracking area\n", t->getA1().getV1(), t->getA1().getV2(),t->getA1().getV3(), t->getA2().getV1(), t->getA2().getV2(), t->getA2().getV3(), t->getA3().getV1(), t->getA3().getV2(), t->getA3().getV3(), t->getA4().getV1(), t->getA4().getV2(), t->getA4().getV3(), t->getB1().getV1(), t->getB1().getV2(), t->getB1().getV3());

/*
    Line *f = new Line(*a1, *a3);
    Line *g = new Line(*b1, *b2);
    Line perp = m->getIntersectionLine(*f, *a2, *g, *(new Vector(0.5, 0.5, 0.5)));
    printf("intersectionLine is [%f, %f, %f]\n", perp.getU().getV1(), perp.getU().getV2(), perp.getU().getV3());

    if(t->inTrackingArea(*a1, *b3, 4, t->getCenter(), m->getEngine())) printf("Contains\n");

    Vector *x = new Vector(5, -2, 0);
    Vector *a = new Vector(-2, -7.5, 2);
    Vector *u = new Vector(-8, 7.5, -2);
    Vector *v = new Vector(2, 10, -2);
    double dist = t->getDistPointPlane(*a, *u, *v, *x);
    printf("distance is %f, should be 1,183.\n", dist);
*/
    m->destroyMatlab();
    return EXIT_SUCCESS;
}

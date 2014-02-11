/*
 * Vector.cpp
 *
 *  Created on: 18.01.2014
 *      Author: daniela
 */
#define _USE_MATH_DEFINES
#include <cmath>
#include "Vector.h"
#include "Line.h"
#include "Matlab.h"
#include "engine.h"
#include <math.h>
#include <iostream>
#include "TrackingArea.h"

using namespace std;

TrackingArea::TrackingArea(Vector a1, Vector a2, Vector a3, Vector a4, Vector b1, Vector b2, Vector b3, Vector b4) {
    this->a1 = a1;
    this->a2 = a2;
    this->a3 = a3;
    this->a4 = a4;
    this->b1 = b1;
    this->b2 = b2;
    this->b3 = b3;
    this->b4 = b4;
    Matlab *m = new Matlab();
    this->center = *(calculateCenter(m->getEngine()));
}

TrackingArea::TrackingArea(Vector* cameraPosition, Vector* cameraDirection, int numberCameras, double maxRange, Engine *ep) {
    setTrackingArea(cameraPosition, cameraDirection, numberCameras, maxRange, ep);
}

Vector TrackingArea::getA1() {
    return this->a1;
}

Vector TrackingArea::getA2() {
    return this->a2;
}

Vector TrackingArea::getA3() {
    return this->a3;
}

Vector TrackingArea::getA4() {
    return this->a4;
}

Vector TrackingArea::getB1() {
    return this->b1;
}

Vector TrackingArea::getB2() {
    return this->b2;
}

Vector TrackingArea::getB3() {
    return this->b3;
}

Vector TrackingArea::getB4() {
    return this->b4;

}
void TrackingArea::setA1(Vector a1) {
    this->a1 = a1;
}
void TrackingArea::setA2(Vector a2) {
    this->a2 = a2;
}

void TrackingArea::setA3(Vector a3) {
    this->a3 = a3;
}

void TrackingArea::setA4(Vector a4) {
    this->a4 = a4;
}

void TrackingArea::setB1(Vector b1) {
    this->b1 = b1;
}
void TrackingArea::setB2(Vector b2) {
    this->b2 = b2;
}

void TrackingArea::setB3(Vector b3) {
    this->b3 = b3;
}

void TrackingArea::setB4(Vector b4) {
    this->b4 = b4;
}

Vector TrackingArea::getCenter() {
    return this->center;
}

void TrackingArea::setCenter(Vector center) {
    this->center = center;
}

Vector* TrackingArea::calculateCenter(Engine *ep) {
    Vector* u1 = new Vector(a1.getV1() - b3.getV1(), a1.getV2() - b3.getV2(), a1.getV3() - b3.getV3());
    Line* diag1 = new Line(b3, *u1);
    Vector* u2 = new Vector(a2.getV1() - b4.getV1(), a2.getV2() - b4.getV2(), a2.getV3() - b4.getV3());
    Line* diag2 = new Line(b4, *u2);
    Vector* result[2];
    Matlab *h = new Matlab(ep);
    h->perpFootTwoLines(*diag1, *diag2, result);
    return result[0];
}

double TrackingArea::getVectorLength(Vector a, Vector b) {
    Vector* diff = new Vector(a.getV1() - b.getV1(), a.getV2() - b.getV2(), a.getV3() - b.getV3());
    return sqrt(pow(diff->getV1(),2) + pow(diff->getV2(), 2) + pow(diff->getV3(), 2));
}

double TrackingArea::getWidth() {
    return getVectorLength(a1, a2);
}

double TrackingArea::getLength() {
    return getVectorLength(a2, a3);
}

double TrackingArea::getHeight() {
    return getVectorLength(a1, b1);
}

double TrackingArea::getDistPointPlane(Vector a, Vector u, Vector v, Vector x) {
    Vector *n = new Vector(u.getV2()*v.getV3()-u.getV3()*v.getV2(), u.getV3()*v.getV1()-u.getV1()*v.getV3(), u.getV1()*v.getV2()- u.getV2()* v.getV1());
    double l = n->getLength();
    double result = (n->scalarMult(x) - n->scalarMult(a))/l;
    if (result < 0) {
        return -result;
    } else {
        return result;
    }
}


/*
 *  checks whether a point x is the TrackingArea or not
 */
bool TrackingArea::contains(Vector x) {
    Vector *u = new Vector(a2.getV1() - a1.getV1(), a2.getV2() - a1.getV2(), a2.getV3() - a1.getV3());
    Vector *v = new Vector(a3.getV1() - a1.getV1(), a3.getV2() - a1.getV2(), a3.getV3() - a1.getV3());
    double diff1 = getDistPointPlane(center, *u, *v, x);
    //u = new Vector(a2.getV1() - a1.getV1(), a2.getV2() - a1.getV2(), a2.getV3() - a1.getV3());
    v = new Vector(b1.getV1() - a1.getV1(), b1.getV2() - a1.getV2(), b1.getV3() - a1.getV3());
    double diff2 = getDistPointPlane(center, *u, *v, x);
    u = new Vector(a3.getV1() - a2.getV1(), a3.getV2() - a2.getV2(), a3.getV3() - a2.getV3());
    v = new Vector(b2.getV1() - a2.getV1(), b2.getV2() - a2.getV2(), b2.getV3() - a2.getV3());
    double diff3 = getDistPointPlane(center, *u, *v, x);
    if ((diff1 <= (getHeight()/2)) && (diff2 <= (getLength()/2)) && (diff3 <= (getWidth()/2))) {
        return true;
    } else {
        return false;
    }
}
// übergebe linie von ursprung auf flacher Ebene und ebene aus drei punkten bestehen mit maxRange abstand
bool TrackingArea::inTrackingArea(Vector cameraPosition, Vector cameraDirection, double maxRange, Vector x, Engine *ep) {
    Matlab *m = new Matlab(ep);
    // finding direction vectors of the plain of the floor of the camera range pyramid.
    Vector n = cameraPosition.add(cameraDirection.mult(maxRange/cameraDirection.getLength()));
    Vector *u = new Vector(n.getV1(), n.getV2(), -(n.getV1()*n.getV1() + n.getV2()*n.getV2())/n.getV3());
    Vector v = n.cross(*u);

    // describing plain by line f and direction vector v
    *u = u->add(n.mult(-1));
    Line *f = new Line(n, *u);

    // describing floor plain by a and b and cameraPosition
    Vector *a = new Vector(n.getV1(), n.getV2(), 0);
    Vector *b = new Vector(n.getV1(), n.getV2() + 0.1, 0);
    Line *g = new Line(cameraPosition, *a);

    Line horizontal = m->getIntersectionLine(*f, v, *g, *b);
    horizontal.setA(n);
    horizontal.setU(n.add(horizontal.getU().mult(-1)));
    printf("horizontal: a is [%f, %f, %f], u is [%f, %f, %f]\n", horizontal.getA().getV1(), horizontal.getA().getV2(), horizontal.getA().getV3(),horizontal.getU().getV1(), horizontal.getU().getV2(), horizontal.getU().getV3());

    v = n.cross(horizontal.getU());
    Line vertical = *(new Line(n, n.add(v.mult(-1))));


  /*
    // a is vector where the camera looks to
    Vector a = cameraDirection.add(cameraPosition);

    printf("a is [%f, %f, %f]", a.getV1(), a.getV2(), a.getV3());

    // b is orthographic on a
    Vector *b = new Vector(a.getV1(), a.getV2(), -(a.getV1()*a.getV1() + a.getV2()*a.getV2())/a.getV3());
    // b and cross are the plain of the quadrat
    Vector cross = a.cross(*b);
    Vector center = cameraPosition.add(cameraDirection.mult(maxRange/cameraDirection.getLength()));

    printf("center is [%f, %f, %f]\n", center.getV1(), center.getV2(), center.getV3());

    *b = b->add(center.mult(-1));
    Line *f = new Line(center, *b);
    // u1 and u2 are on the floor plain vectors, that should build the floor plain.

    Vector *u1 = new Vector(cameraDirection.getV1(), cameraDirection.getV2(), 0);
    Vector *u2 = new Vector(cameraDirection.getV2(), cameraDirection.getV2() + 0.1, 0);

    printf("u1 :[%f, %f, %f]\n", u1->getV1(), u1->getV2(), u1->getV3());

    Line *g = new Line(cameraPosition, *u1);
    // u is one middle line of the quadrat, v is vector of cameraPosition to quadrat, c is crossproduct
    Line u = m->getIntersectionLine(*f, cross.add(center), *g, *u2);

    printf("f: a is [%f, %f, %f], u is [%f, %f, %f]\n",f->getA().getV1(), f->getA().getV2(), f->getA().getV3(),f->getU().getV1(), f->getU().getV2(), f->getU().getV3());
    printf("g: a is [%f, %f, %f], u is [%f, %f, %f]\n",g->getA().getV1(), g->getA().getV2(), g->getA().getV3(),g->getU().getV1(), g->getU().getV2(), g->getU().getV3());



    Vector v = cameraPosition.add(a.mult(-1));
    Vector c = u.getU().cross(v);
    c = c.add(center);
    // c and v are the vectors of the lines of the quadrat
*/
    double maxLengthHorizontal = maxRange;
    double hypotenuse = maxRange / cos(28.5 * M_PI / 180);
    double maxDiffHorizontal = hypotenuse * sin(28.5 * M_PI / 180);
    // Line *horizontal = new Line(a, v);
    // perp is Vector on line *horizontal

    Vector perp = m->perpFootOneLine(horizontal, x);
    double diff = (perp.add(x.mult(-1))).getLength();
    double length = (perp.add(cameraPosition.mult(-1))).getLength();

    if ((length > maxLengthHorizontal) || (length < 0) || (diff/length > maxDiffHorizontal/maxLengthHorizontal)) {
        printf("length: %f, maxLengthHorizontal: %f, diff: %f, maxDiffHorizontal: %f\n", length, maxLengthHorizontal, diff, maxDiffHorizontal);
        return false;
    } else {
        double maxLengthVertical = maxRange;
        hypotenuse = maxRange / cos(21.5 * M_PI/180);
        double maxDiffVertical = maxRange * sin(21.5 * M_PI / 180);
        //Line *vertical = new Line(a, u.getU());

        perp = m->perpFootOneLine(vertical, x);
        diff = (perp.add(x.mult(-1))).getLength();
        length = (perp.add(cameraPosition.mult(-1))).getLength();

        if ((length > maxLengthVertical) || (length < 0) || (diff/length > maxDiffVertical/maxLengthVertical)) {
            printf("length: %f, maxLengthVertical: %f, diff: %f, maxDiffVertical: %f\n", length, maxLengthVertical, diff, maxDiffVertical);
            return false;
        }
        else {
            return true;
        }
    }

    /*
     *old version of cone
    Vector *u = new Vector(cameraDirection.add(cameraPosition.mult(-1)));
    Line *direct = new Line(&cameraPosition, u);
    Vector perp = m->perpFootOneLine(*direct, x);
    Vector dist = perp.add(x.mult(-1));
    // diff is the distance between x and the cameraDirection line
    // diff / length(perp - cameraPosition) mustn't be greater than maxDiff/maxLength
    double diff = dist.getLength();
    double length = (perp.add(cameraPosition.mult(-1))).getLength();
    double maxLength = maxRange * cos(21.5 * M_PI / 180);
    double maxDiff = maxRange * sin(21.5 * M_PI / 180);
    if ((length > maxLength) || (length < 0)) {
        return false;
    } else if (diff/length > maxDiff/maxRange) {
        return false;
    }
    return true;*/
}


/*
 * checks whether all cameras observe x
 */
bool TrackingArea::inCameraRange(Vector *cameraPosition, Vector* cameraDirection, int numberCameras, double maxRange, Vector x, Engine *ep) {
    // idea: cameraPosition[i] + r*cameraDirection[i]/cameraDirection[i].getLength + s * n + t * m = x
    // vectors (cameraDirection[i], n, m) are an orthogonalsystem of B = (cameraDirection, (1 0 0), (0 1 0) calculated by gram-schmidt
    // m, n are normalized
    // r in [0, maxRange]

    //Vector* a = new Vector(1, 0, 0);
    //Vector* b = new Vector(0, 1, 0);
    for (int i = 0; i < numberCameras; i++) {
        /*
        // n = a - (cameraDirection[i] * a)/(cameraDirection[i] * cameraDirection[i]) * cameraDirection[i]
        double q = cameraDirection[i].scalarMult(*a)/cameraDirection[i].getLength();
        Vector n = a->add(cameraDirection[i].mult(-q));
        // m = b - (cameraDirection[i] * b)/(cameraDirection[i] * cameraDirection[i]) * cameraDirection[i] - (n * b)/(n * n) * n
        q = cameraDirection[i].scalarMult(*b)/cameraDirection[i].getLength();
        double r = n.scalarMult(*b)/n.getLength();
        Vector m = b->add(cameraDirection[i].mult(-q));
        m = m.add(n.mult(-r));
        // normalize
        n = n.mult(1/n.getLength());
        m = m.mult(1/m.getLength());*/
        if (inTrackingArea(cameraPosition[i], cameraDirection[i], maxRange, x, ep) == false) {
            printf("camPos[%f, %f, %f], x: [%f, %f, %f]\n", cameraPosition[i].getV1(), cameraPosition[i].getV2(), cameraPosition[i].getV3(), x.getV1(), x.getV2(), x.getV3());
            return false;
        }
    }
    return true;
}

/*
 *  calculates the maximum TrackingArea in form of a quader
 */
void TrackingArea::setTrackingArea(Vector* cameraPosition, Vector* cameraDirection, int numberCameras, double maxRange, Engine *ep) {
    double v1 = 0;
    double v2 = 0;
    double v3 = 0;
    // normalize cameraDirection Vectors

    for (int i = 0; i < numberCameras; i++) {
        cameraDirection[i] = cameraDirection[i].mult(cameraDirection[i].getLength());
    }
    /*for (int i = 0; i < numberCameras; i++) {
        // cameraPosition[i] + maxRange * cameraDirection[i]
        v1 = v1 + cameraPosition[i].getV1() + maxRange * cameraDirection[i].getV1();
        v2 = v2 + cameraPosition[i].getV2() + maxRange * cameraDirection[i].getV2();
        v3 = v3 + cameraPosition[i].getV3() + maxRange * cameraDirection[i].getV3();
    }
    v1 = v1 / numberCameras;
    v2 = v2 / numberCameras;
    v3 = v3 / numberCameras;*/

   /* Matlab *m = new Matlab(ep);
    Line **cameraLines = new Line*[numberCameras];
    cout << "test1" << endl;
    for (int i = 0; i < numberCameras; i++) {
        *cameraLines[i] = *(new Line(cameraPosition[i], cameraDirection[i].add(cameraPosition[i].mult(-1))));
        cout << "test" << endl;
    }
    Vector center = m->interpolateLines(cameraLines, numberCameras);
    printf("center is [%f, %f, %f]\n", center.getV1(), center.getV2(), center.getV3());*/
    Vector center = *(new Vector(0.5, 0.5, 0.5));

    setCenter(center);
    setA1(center);
    setA2(center);
    setA3(center);
    setA4(center);
    setB1(center);
    setB2(center);
    setB3(center);
    setB4(center);
    double posChange = 0.01;
    while (inCameraRange(cameraPosition, cameraDirection, numberCameras, maxRange, a1, ep) && inCameraRange(cameraPosition, cameraDirection, numberCameras, maxRange, a2, ep)
            && inCameraRange(cameraPosition, cameraDirection, numberCameras, maxRange, a3, ep) && inCameraRange(cameraPosition, cameraDirection, numberCameras, maxRange, a4, ep)
            && inCameraRange(cameraPosition, cameraDirection, numberCameras, maxRange, b1, ep) && inCameraRange(cameraPosition, cameraDirection, numberCameras, maxRange, b2, ep)
            && inCameraRange(cameraPosition, cameraDirection, numberCameras, maxRange, b3, ep) && inCameraRange(cameraPosition, cameraDirection, numberCameras, maxRange, b4, ep)) {
        setA1(*(new Vector(center.getV1() - posChange, center.getV2() - posChange, center.getV3() - posChange)));
        setA2(*(new Vector(center.getV1() - posChange, center.getV2() - posChange, center.getV3() + posChange)));
        setA3(*(new Vector(center.getV1() - posChange, center.getV2() + posChange, center.getV3() + posChange)));
        setA4(*(new Vector(center.getV1() - posChange, center.getV2() + posChange, center.getV3() - posChange)));
        setB1(*(new Vector(center.getV1() + posChange, center.getV2() - posChange, center.getV3() - posChange)));
        setB2(*(new Vector(center.getV1() + posChange, center.getV2() - posChange, center.getV3() + posChange)));
        setB3(*(new Vector(center.getV1() + posChange, center.getV2() + posChange, center.getV3() + posChange)));
        setB4(*(new Vector(center.getV1() + posChange, center.getV2() + posChange, center.getV3() - posChange)));
        posChange += 0.01;
    }
    posChange -= 0.01;
    setA1(*(new Vector(center.getV1() - posChange, center.getV2() - posChange, center.getV3() - posChange)));
    setA2(*(new Vector(center.getV1() - posChange, center.getV2() - posChange, center.getV3() + posChange)));
    setA3(*(new Vector(center.getV1() - posChange, center.getV2() + posChange, center.getV3() + posChange)));
    setA4(*(new Vector(center.getV1() - posChange, center.getV2() + posChange, center.getV3() - posChange)));
    setB1(*(new Vector(center.getV1() + posChange, center.getV2() - posChange, center.getV3() - posChange)));
    setB2(*(new Vector(center.getV1() + posChange, center.getV2() - posChange, center.getV3() + posChange)));
    setB3(*(new Vector(center.getV1() + posChange, center.getV2() + posChange, center.getV3() + posChange)));
    setB4(*(new Vector(center.getV1() + posChange, center.getV2() + posChange, center.getV3() - posChange)));
}

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
#include "TrackingArea.h"

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
    Line* diag1 = new Line(&b3, u1);
    Vector* u2 = new Vector(a2.getV1() - b4.getV1(), a2.getV2() - b4.getV2(), a2.getV3() - b4.getV3());
    Line* diag2 = new Line(&b4, u2);
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

double TrackingArea::getDistPointPlane(Vector a1, Vector u, Vector v, Vector x) {
    Vector *n = new Vector(u.getV2()*v.getV3()-u.getV3()*v.getV2(), u.getV3()*v.getV1()-u.getV1()*v.getV3(), u.getV1()*v.getV2()- u.getV2()* v.getV1());
    Vector *h = new Vector(0, 0, 0);
    double l = getVectorLength(*n, *h);
    double a = n->getV1()*a1.getV1()+n->getV2()*a2.getV2()+n->getV3()*a2.getV3();
    double result = (n->getV1()*x.getV1()+n->getV2()*x.getV2()+n->getV3()*x.getV3()-a)/l;
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

bool TrackingArea::inTrackingArea(Vector cameraPosition, Vector cameraDirection, double maxRange, Vector x, Engine *ep) {
    /*cameraPosition.putVariable("p", ep);
    Vector d = cameraDirection.mult(1/cameraDirection.getLength());
    d.putVariable("d", ep);
    n.putVariable("n", ep);
    m.putVariable("m", ep);
    x.putVariable("x", ep);
    engEvalString(ep, "b = x - p");
    engEvalString(ep, "A = [d1 n1 m1; d2 n2 m2; d3 n3 m3]");
    // x = (r, s, t)
    engEvalString(ep, "x = inv(A) * b");
    mxArray *result;
    result = engGetVariable(ep, "x(1)");
    double r = mxGetPr(result)[0];
    double s = mxGetPr(result)[1];
    double t = mxGetPr(result)[2];
    if ((r > maxRange) || (r < 0)) {
        return false
    } //else if (r*r + s*s > )*/
    Matlab *m = new Matlab(ep);
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
    return true;
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
    for (int i = 0; i < numberCameras; i++) {
        // cameraPosition[i] + maxRange * cameraDirection[i]
        v1 = v1 + cameraPosition[i].getV1() + maxRange * cameraDirection[i].getV1();
        v2 = v2 + cameraPosition[i].getV2() + maxRange * cameraDirection[i].getV2();
        v3 = v3 + cameraPosition[i].getV3() + maxRange * cameraDirection[i].getV3();
    }
    v1 = v1 / numberCameras;
    v2 = v2 / numberCameras;
    v3 = v3 / numberCameras;
    Vector *center = new Vector(v1, v2, v3);
    setCenter(* center);
    setA1(*center);
    setA2(*center);
    setA3(*center);
    setA4(*center);
    setB1(*center);
    setB2(*center);
    setB3(*center);
    setB4(*center);
    double posChange = 0.1;
    while (inCameraRange(cameraPosition, cameraDirection, numberCameras, maxRange, a1, ep) && inCameraRange(cameraPosition, cameraDirection, numberCameras, maxRange, a2, ep)
            && inCameraRange(cameraPosition, cameraDirection, numberCameras, maxRange, a3, ep) && inCameraRange(cameraPosition, cameraDirection, numberCameras, maxRange, a4, ep)
            && inCameraRange(cameraPosition, cameraDirection, numberCameras, maxRange, b1, ep) && inCameraRange(cameraPosition, cameraDirection, numberCameras, maxRange, b2, ep)
            && inCameraRange(cameraPosition, cameraDirection, numberCameras, maxRange, b3, ep) && inCameraRange(cameraPosition, cameraDirection, numberCameras, maxRange, b4, ep)) {
        setA1(*(new Vector(center->getV1() - posChange, center->getV2() - posChange, center->getV3() - posChange)));
        setA2(*(new Vector(center->getV1() - posChange, center->getV2() - posChange, center->getV3() + posChange)));
        setA3(*(new Vector(center->getV1() - posChange, center->getV2() + posChange, center->getV3() + posChange)));
        setA4(*(new Vector(center->getV1() - posChange, center->getV2() + posChange, center->getV3() - posChange)));
        setB1(*(new Vector(center->getV1() + posChange, center->getV2() - posChange, center->getV3() - posChange)));
        setB2(*(new Vector(center->getV1() + posChange, center->getV2() - posChange, center->getV3() + posChange)));
        setB3(*(new Vector(center->getV1() + posChange, center->getV2() + posChange, center->getV3() + posChange)));
        setB4(*(new Vector(center->getV1() + posChange, center->getV2() + posChange, center->getV3() - posChange)));
        posChange += 0.1;
    }
    setA1(*(new Vector(center->getV1() - posChange, center->getV2() - posChange, center->getV3() - posChange)));
    setA2(*(new Vector(center->getV1() - posChange, center->getV2() - posChange, center->getV3() + posChange)));
    setA3(*(new Vector(center->getV1() - posChange, center->getV2() + posChange, center->getV3() + posChange)));
    setA4(*(new Vector(center->getV1() - posChange, center->getV2() + posChange, center->getV3() - posChange)));
    setB1(*(new Vector(center->getV1() + posChange, center->getV2() - posChange, center->getV3() - posChange)));
    setB2(*(new Vector(center->getV1() + posChange, center->getV2() - posChange, center->getV3() + posChange)));
    setB3(*(new Vector(center->getV1() + posChange, center->getV2() + posChange, center->getV3() + posChange)));
    setB4(*(new Vector(center->getV1() + posChange, center->getV2() + posChange, center->getV3() - posChange)));
}

/*
 * Vector.cpp
 *
 *  Created on: 18.01.2014
 *      Author: daniela
 */

#include "Vector.h"
#include "Line.h"
#include "Matlab.h"
#include "engine.h"
#include "math.h"
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

Vector* TrackingArea::getCenter(Engine *ep) {
    Vector* u1 = new Vector(a1.getV1() - b3.getV1(), a1.getV2() - b3.getV2(), a1.getV3() - b3.getV3());
    Line* diag1 = new Line(&b3, u1);
    Vector* u2 = new Vector(a2.getV1() - b4.getV1(), a2.getV2() - b4.getV2(), a2.getV3() - b4.getV3());
    Line* diag2 = new Line(&b4, u2);
    Vector* result[2];
    Matlab *h = new Matlab();
    h->perpFootTwoLines(*diag1, *diag2, ep, result);
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

bool TrackingArea::isInTrackingArea(Vector x, Engine *ep) {
    Vector* center = getCenter(ep);
    Vector *u = new Vector(a2.getV1() - a1.getV1(), a2.getV2() - a1.getV2(), a2.getV3() - a1.getV3());
    Vector *v = new Vector(a3.getV1() - a1.getV1(), a3.getV2() - a1.getV2(), a3.getV3() - a1.getV3());
    double diff1 = getDistPointPlane(*center, *u, *v, x);
    //u = new Vector(a2.getV1() - a1.getV1(), a2.getV2() - a1.getV2(), a2.getV3() - a1.getV3());
    v = new Vector(b1.getV1() - a1.getV1(), b1.getV2() - a1.getV2(), b1.getV3() - a1.getV3());
    double diff2 = getDistPointPlane(*center, *u, *v, x);
    u = new Vector(a3.getV1() - a2.getV1(), a3.getV2() - a2.getV2(), a3.getV3() - a2.getV3());
    v = new Vector(b2.getV1() - a2.getV1(), b2.getV2() - a2.getV2(), b2.getV3() - a2.getV3());
    double diff3 = getDistPointPlane(*center, *u, *v, x);
    if ((diff1 <= (getHeight()/2)) && (diff2 <= (getLength()/2)) && (diff3 <= (getWidth()/2))) {
        return true;
    } else {
        return false;
    }
}

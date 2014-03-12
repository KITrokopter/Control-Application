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
#include <stdio.h>
#include "TrackingArea.h"
#include <ros/ros.h>

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

TrackingArea::TrackingArea(vector<Vector> cameraPosition, vector<Vector> cameraDirection, int numberCameras, double maxRange, Engine *ep) {
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

/*
 * calculates the distance of E: a + r*u + s*v and a point x
 */
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
 * calculates perpendicular point of point x and plane E: a + ru + sv
 */
Vector TrackingArea::getPerpPointPlane(Vector a, Vector u, Vector v, Vector x) {
    // n * x = n * a => as x + t*n is the perpendicular point it has to be: n * (x + t*n) = n * a <=> = n*a/(n*x*n.getLength())
    Vector *n = new Vector(u.getV2()*v.getV3()-u.getV3()*v.getV2(), u.getV3()*v.getV1()-u.getV1()*v.getV3(), u.getV1()*v.getV2()- u.getV2()* v.getV1());
    double d = n->scalarMult(a)-(n->scalarMult(x));
    double e = n->getV1()*n->getV1() + n->getV2()*n->getV2() + n->getV3()*n->getV3();
    double t = (d/e);
    Vector perp = x.add(n->mult(t));
    return perp;
}


/*
 *  checks whether a point x is the TrackingArea or not
 */
bool TrackingArea::contains(Vector x) {
    Vector *u = new Vector(a2.getV1() - a1.getV1(), a2.getV2() - a1.getV2(), a2.getV3() - a1.getV3());
    Vector *v = new Vector(a3.getV1() - a1.getV1(), a3.getV2() - a1.getV2(), a3.getV3() - a1.getV3());
    double diff1 = getDistPointPlane(center, *u, *v, x);
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
    // center point of the floor of the pyramid
    Vector n = cameraPosition.add(cameraDirection.mult(maxRange/cameraDirection.getLength()));
    // finding direction vectors of the plain of the floor of the camera range pyramid.
    Vector *u = new Vector(cameraDirection.getV1(), cameraDirection.getV2(), -(cameraDirection.getV1()*cameraDirection.getV1() + cameraDirection.getV2()*cameraDirection.getV2())/cameraDirection.getV3());
    Vector v = cameraDirection.cross(*u);

    // describing plain by line f and direction vector v
    Line *f = new Line(cameraDirection, *u);

    // describing floor plain by a and b and cameraPosition
    Vector *a = new Vector(cameraDirection.getV1(), cameraDirection.getV2(), 0);
    Vector *b = new Vector(cameraDirection.getV1(), cameraDirection.getV2() + 0.1, 0);
    Line *g = new Line(cameraPosition, *a);

    Line horizontal = m->getIntersectionLine(*f, v, *g, *b);

    v = cameraDirection.cross(horizontal.getU());

    horizontal.setA(n);

    Line vertical = *(new Line(n, v));

    double maxLengthHorizontal = maxRange;
    double hypotenuse = maxRange / cos(28.5 * M_PI / 180);
    double maxDiffHorizontal = hypotenuse * sin(28.5 * M_PI / 180);

    Vector perp = getPerpPointPlane(horizontal.getA(), horizontal.getU(), cameraDirection, x);
    double diff = (perp.add(x.mult(-1))).getLength();
    double length = (perp.add(cameraPosition.mult(-1))).getLength();

    if ((length > maxLengthHorizontal) || (length < 0) || (diff/length > maxDiffHorizontal/maxLengthHorizontal)) {
        return false;
    } else {
        double maxLengthVertical = maxRange;
        hypotenuse = maxRange / cos(21.5 * M_PI/180);
        double maxDiffVertical = maxRange * sin(21.5 * M_PI / 180);

        perp = getPerpPointPlane(vertical.getA(), vertical.getU(), cameraDirection, x);
        diff = (perp.add(x.mult(-1))).getLength();
        length = (perp.add(cameraPosition.mult(-1))).getLength();

        if ((length > maxLengthVertical) || (length < 0) || (diff/length > maxDiffVertical/maxLengthVertical)) {
            return false;
        }
        else {
            return true;
        }
    }
}


/*
 * checks whether all cameras observe x
 */
bool TrackingArea::inCameraRange(std::vector<Vector> cameraPosition, std::vector<Vector> cameraDirection, int numberCameras, double maxRange, Vector x, Engine *ep) {
    for (int i = 0; i < numberCameras; i++) {
        if (inTrackingArea(cameraPosition[i], cameraDirection[i], maxRange, x, ep) == false) {
            return false;
        }
    }
    return true;
}

void TrackingArea::increaseTrackingArea(double posChange) {
    Vector center = getCenter();
    setA1(Vector(center.getV1() - posChange, center.getV2() - posChange, center.getV3() - posChange));
    setA2(Vector(center.getV1() - posChange, center.getV2() - posChange, center.getV3() + posChange));
    setA3(Vector(center.getV1() - posChange, center.getV2() + posChange, center.getV3() + posChange));
    setA4(Vector(center.getV1() - posChange, center.getV2() + posChange, center.getV3() - posChange));
    setB1(Vector(center.getV1() + posChange, center.getV2() - posChange, center.getV3() - posChange));
    setB2(Vector(center.getV1() + posChange, center.getV2() - posChange, center.getV3() + posChange));
    setB3(Vector(center.getV1() + posChange, center.getV2() + posChange, center.getV3() + posChange));
    setB4(Vector(center.getV1() + posChange, center.getV2() + posChange, center.getV3() - posChange));
}

void TrackingArea::increaseTrackingArea(double height, double posChange) {
    Vector center = getCenter();
    setA1(Vector(center.getV1() - posChange, center.getV2() - posChange, center.getV3() - height));
    setA2(Vector(center.getV1() - posChange, center.getV2() - posChange, center.getV3() + height));
    setA3(Vector(center.getV1() - posChange, center.getV2() + posChange, center.getV3() + height));
    setA4(Vector(center.getV1() - posChange, center.getV2() + posChange, center.getV3() - height));
    setB1(Vector(center.getV1() + posChange, center.getV2() - posChange, center.getV3() - height));
    setB2(Vector(center.getV1() + posChange, center.getV2() - posChange, center.getV3() + height));
    setB3(Vector(center.getV1() + posChange, center.getV2() + posChange, center.getV3() + height));
    setB4(Vector(center.getV1() + posChange, center.getV2() + posChange, center.getV3() - height));
}

/*
 *  calculates the maximum TrackingArea in form of a quader
 */
void TrackingArea::setTrackingArea(std::vector<Vector> cameraPosition, std::vector<Vector> cameraDirection, int numberCameras, double maxRange, Engine *ep) {
    Matlab *m = new Matlab(ep);
    Line *cameraLines = new Line[numberCameras];
    for (int i = 0; i < numberCameras; i++) {
        cameraLines[i] = Line();
        cameraLines[i] = Line(cameraPosition[i], cameraDirection[i]);
    }
    Vector center = m->interpolateLines(cameraLines, numberCameras);
    ROS_DEBUG("center is [%f, %f, %f]", center.getV1(), center.getV2(), center.getV3());
    setCenter(center);
    setA1(center);
    setA2(center);
    setA3(center);
    setA4(center);
    setB1(center);
    setB2(center);
    setB3(center);
    setB4(center);
    double posChange = 1;

    // searching border of tracking area
    while (inCameraRange(cameraPosition, cameraDirection, numberCameras, maxRange, a1, ep) && inCameraRange(cameraPosition, cameraDirection, numberCameras, maxRange, a2, ep)
            && inCameraRange(cameraPosition, cameraDirection, numberCameras, maxRange, a3, ep) && inCameraRange(cameraPosition, cameraDirection, numberCameras, maxRange, a4, ep)
            && inCameraRange(cameraPosition, cameraDirection, numberCameras, maxRange, b1, ep) && inCameraRange(cameraPosition, cameraDirection, numberCameras, maxRange, b2, ep)
            && inCameraRange(cameraPosition, cameraDirection, numberCameras, maxRange, b3, ep) && inCameraRange(cameraPosition, cameraDirection, numberCameras, maxRange, b4, ep)) {
        posChange *= 2;
        increaseTrackingArea(posChange);
        ROS_DEBUG("increasing, cube size: %f", 2 * posChange);
    }

    // border is between leftBorder and rightBorder
    double leftBorder = posChange/2;
    double rightBorder = posChange;
    double middle = (rightBorder - leftBorder)/2;

    bool tracked = false;
    // searching exact border of tracking area
    while ((rightBorder - leftBorder > 5) && (tracked == false)) {

        // checks whether all corners of tracking area are still tracked of all cameras
        if (inCameraRange(cameraPosition, cameraDirection, numberCameras, maxRange, a1, ep) && inCameraRange(cameraPosition, cameraDirection, numberCameras, maxRange, a2, ep)
            && inCameraRange(cameraPosition, cameraDirection, numberCameras, maxRange, a3, ep) && inCameraRange(cameraPosition, cameraDirection, numberCameras, maxRange, a4, ep)
            && inCameraRange(cameraPosition, cameraDirection, numberCameras, maxRange, b1, ep) && inCameraRange(cameraPosition, cameraDirection, numberCameras, maxRange, b2, ep)
            && inCameraRange(cameraPosition, cameraDirection, numberCameras, maxRange, b3, ep) && inCameraRange(cameraPosition, cameraDirection, numberCameras, maxRange, b4, ep)) {

            // border is between leftBorder and middle
            rightBorder = middle;
            tracked = true;
        } else {
            // border is between middle and rightBorder
            leftBorder = middle;
            tracked = false;
        }
        middle = (rightBorder - leftBorder)/2;
        increaseTrackingArea(middle);
        ROS_DEBUG("binary search, cube size: %f", 2 * middle);
    }

    // border is (rightBorder - leftBorder)/2
    ROS_DEBUG("maximal cube size is %f", middle);



}

void TrackingArea::printTrackingArea() {
    printf("Tracking area is from [%f, %f, %f] to [%f, %f, %f], cube is of size %f.\n", a1.getV1(), a1.getV2(), a1.getV3(), b3.getV1(), b3.getV2(), b3.getV3(), a1.add(a2.mult(-1)).getLength());
}

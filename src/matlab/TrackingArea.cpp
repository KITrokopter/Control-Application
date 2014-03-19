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
/*
TrackingArea::TrackingArea(Vector a1, Vector a2, Vector a3, Vector a4, Vector low, Vector up) {
    this->a1 = a1;
    this->a2 = a2;
    this->a3 = a3;
    this->a4 = a4;
    this->low = low;
    this->up = up;
    Matlab *m = new Matlab();
    this->center = *(calculateCenter(m->getEngine()));
}
*/
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

Vector TrackingArea::getLow() {
    return this->low;
}

Vector TrackingArea::getUp() {
    return this->up;
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

void TrackingArea::setLow(Vector low) {
    this->low = low;
}

void TrackingArea::setUp(Vector up) {
    this->up = up;
}

Vector TrackingArea::getCenter() {
    return this->center;
}

void TrackingArea::setCenter(Vector center) {
    this->center = center;
}
/*
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
*/

double TrackingArea::getWidth() {
    return a1.add(a2.mult(-1)).getLength();
}

double TrackingArea::getLength() {
    return a2.add(a3.mult(-1)).getLength();
}

double TrackingArea::getHeight() {
    return (up.getV3() - low.getV3());
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
    /*
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
    }*/
    return true;
}

// übergebe linie von ursprung auf flacher Ebene und ebene aus drei punkten bestehen mit maxRange abstand
bool TrackingArea::inTrackingArea(Vector cameraPosition, Vector cameraDirection, double maxRange, Vector x, Engine *ep) {
    Matlab *m = new Matlab(ep);
    // center point of the floor of the pyramidb1
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

void TrackingArea::increaseTrackingArea(double height) {
    Vector center = getCenter();
    setA1(Vector(center.getV1(), center.getV2(), center.getV3() + height));
    setA2(Vector(center.getV1(), center.getV2(), center.getV3() + height));
    setA3(Vector(center.getV1(), center.getV2(), center.getV3() + height));
    setA4(Vector(center.getV1(), center.getV2(), center.getV3() + height));
    setUp(Vector(center.getV1(), center.getV2(), center.getV3() + height));
    setLow(Vector(center.getV1(), center.getV2(), center.getV3() + height));
}

void TrackingArea::increaseTrackingArea(double posChange, double heightPos, double heightNeg) {
    Vector center = getCenter();
    setA1(Vector(center.getV1() - posChange, center.getV2() - posChange, center.getV3()));
    setA2(Vector(center.getV1() - posChange, center.getV2() + posChange, center.getV3()));
    setA3(Vector(center.getV1() + posChange, center.getV2() + posChange, center.getV3()));
    setA4(Vector(center.getV1() + posChange, center.getV2() - posChange, center.getV3()));
    setUp(Vector(center.getV1(), center.getV2(), center.getV3() + heightPos));
    setLow(Vector(center.getV1(), center.getV2(), center.getV3() - heightNeg));
}

/*
double TrackingArea::getVolume(double height) {
    double upperLength = b1.add(b2.mult(-1)).getLength();
    double lowerLength = a1.add(a2.mult(-1)).getLength();
    double upperVolume = upperLength * upperLength;
    double lowerVolume = lowerLength * lowerLength;
    double volume = 1.0/3 * height * (upperVolume + lowerVolume + sqrt(upperVolume * lowerVolume));
    return volume;
}*/

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
    if (!(inCameraRange(cameraPosition, cameraDirection, numberCameras, maxRange, center, ep))) {
        ROS_ERROR("center isn't tracked, maximal range is too small!");
    } else {

        setCenter(center);
        setA1(center);
        setA2(center);
        setA3(center);
        setA4(center);
        setUp(center);
        setLow(center);
        double posChange = 1;

        // searching side border of tracking area
        while (inCameraRange(cameraPosition, cameraDirection, numberCameras, maxRange, a1, ep) && inCameraRange(cameraPosition, cameraDirection, numberCameras, maxRange, a2, ep)
                && inCameraRange(cameraPosition, cameraDirection, numberCameras, maxRange, a3, ep) && inCameraRange(cameraPosition, cameraDirection, numberCameras, maxRange, a4, ep)) {
            posChange *= 2;
            increaseTrackingArea(posChange, 0, 0);
            ROS_DEBUG("increasing, side size of center: %f", 2 * posChange);
        }

        // border is between leftBorder and rightBorder
        double leftBorder = posChange/2;
        double rightBorder = posChange;
        double middle = leftBorder + (rightBorder - leftBorder)/2;
        increaseTrackingArea(middle, 0, 0);

        bool tracked = false;
        double sideBorder = leftBorder;
        // searching exact border of tracking area
        while ((rightBorder - leftBorder > 5) || (tracked == false)) {

            // checks whether all corners of tracking area are still tracked of all cameras
            if (inCameraRange(cameraPosition, cameraDirection, numberCameras, maxRange, a1, ep) && inCameraRange(cameraPosition, cameraDirection, numberCameras, maxRange, a2, ep)
                && inCameraRange(cameraPosition, cameraDirection, numberCameras, maxRange, a3, ep) && inCameraRange(cameraPosition, cameraDirection, numberCameras, maxRange, a4, ep)) {

                // border is between middle and rightBorder
                leftBorder = middle;
                tracked = true;
                ROS_DEBUG("TRUE");
                sideBorder = middle;
            } else {
                // border is between leftBorder and middle
                rightBorder = middle;
                tracked = false;
                ROS_DEBUG("false");
            }
            middle = leftBorder + (rightBorder - leftBorder)/2;
            increaseTrackingArea(middle, 0, 0);
            ROS_DEBUG("binary search, side size: %f", 2 * middle);
        }

        double newSideBorder = sideBorder;
        double lower = 1;
        while (sideBorder <= newSideBorder) {
            sideBorder = newSideBorder;
            increaseTrackingArea(-1 * lower);
            ROS_DEBUG("moving center down: %f", lower);
            posChange = 1;

            // searching side border of tracking area
            while (inCameraRange(cameraPosition, cameraDirection, numberCameras, maxRange, a1, ep) && inCameraRange(cameraPosition, cameraDirection, numberCameras, maxRange, a2, ep)
                    && inCameraRange(cameraPosition, cameraDirection, numberCameras, maxRange, a3, ep) && inCameraRange(cameraPosition, cameraDirection, numberCameras, maxRange, a4, ep)) {
                posChange *= 2;
                increaseTrackingArea(posChange, 0, 0);
                ROS_DEBUG("increasing, side size of center: %f", 2 * posChange);
            }

            // border is between leftBorder and rightBorder
            leftBorder = posChange/2;
            rightBorder = posChange;
            middle = leftBorder + (rightBorder - leftBorder)/2;
            increaseTrackingArea(middle, 0, 0);

            tracked = false;
            newSideBorder = leftBorder;
            // searching exact border of tracking area
            while ((rightBorder - leftBorder > 5) || (tracked == false)) {

                // checks whether all corners of tracking area are still tracked of all cameras
                if (inCameraRange(cameraPosition, cameraDirection, numberCameras, maxRange, a1, ep) && inCameraRange(cameraPosition, cameraDirection, numberCameras, maxRange, a2, ep)
                    && inCameraRange(cameraPosition, cameraDirection, numberCameras, maxRange, a3, ep) && inCameraRange(cameraPosition, cameraDirection, numberCameras, maxRange, a4, ep)) {

                    // border is between middle and rightBorder
                    leftBorder = middle;
                    tracked = true;
                    ROS_DEBUG("TRUE");
                    newSideBorder = middle;
                } else {
                    // border is between leftBorder and middle
                    rightBorder = middle;
                    tracked = false;
                    ROS_DEBUG("false");
                }
                middle = leftBorder + (rightBorder - leftBorder)/2;
                increaseTrackingArea(middle, 0, 0);
                ROS_DEBUG("binary search, side size: %f", 2 * middle);
            }
            lower++;
        }



        ROS_DEBUG("maximal quadrat size is %f at height %f", sideBorder * 2, -lower);

















        posChange = 1;
        // searching upper border of tracking area
        while (inCameraRange(cameraPosition, cameraDirection, numberCameras, maxRange, up, ep)) {
            posChange *= 2;
            increaseTrackingArea(0, posChange, 0);
            ROS_DEBUG("increasing, upper size of center: %f", posChange);
        }

        // border is between leftBorder and rightBorder
        leftBorder = posChange/2;
        rightBorder = posChange;
        middle = leftBorder + (rightBorder - leftBorder)/2;
        increaseTrackingArea(0, middle, 0);

        tracked = false;
        double upperBorder = leftBorder;
        // searching exact border of tracking area
        while ((rightBorder - leftBorder > 5) || (tracked == false)) {

            // checks whether all corners of tracking area are still tracked of all cameras
            if (inCameraRange(cameraPosition, cameraDirection, numberCameras, maxRange, up, ep)) {

                // border is between middle and rightBorder
                leftBorder = middle;
                tracked = true;
                ROS_DEBUG("TRUE");
                upperBorder = middle;
            } else {
                // border is between leftBorder and middle
                rightBorder = middle;
                tracked = false;
                ROS_DEBUG("false");
            }
            middle = leftBorder + (rightBorder - leftBorder)/2;
            increaseTrackingArea(0, middle, 0);
            ROS_DEBUG("binary search, upper size: %f", middle);
        }

        ROS_DEBUG("maximal upper size is %f", upperBorder);








        posChange = 1;
        // searching lower border of tracking area
        while (inCameraRange(cameraPosition, cameraDirection, numberCameras, maxRange, low, ep)) {
            posChange *= 2;
            increaseTrackingArea(0, 0, posChange);
            ROS_DEBUG("increasing, lower size of center: %f", posChange);
        }

        // border is between leftBorder and rightBorder
        leftBorder = posChange/2;
        rightBorder = posChange;
        middle = leftBorder + (rightBorder - leftBorder)/2;
        increaseTrackingArea(0, 0, middle);

        tracked = false;
        double lowerBorder = 0;
        // searching exact border of tracking area
        while ((rightBorder - leftBorder > 5) || (tracked == false)) {

            // checks whether all corners of tracking area are still tracked of all cameras
            if (inCameraRange(cameraPosition, cameraDirection, numberCameras, maxRange, low, ep)) {

                // border is between middle and rightBorder
                leftBorder = middle;
                tracked = true;
                ROS_DEBUG("TRUE");
                lowerBorder = middle;
            } else {
                // border is between leftBorder and middle
                rightBorder = middle;
                tracked = false;
                ROS_DEBUG("false");
            }
            middle = leftBorder + (rightBorder - leftBorder)/2;
            increaseTrackingArea(0, 0, middle);
            ROS_DEBUG("binary search, lower size: %f", middle);
        }

        ROS_DEBUG("maximal lower size is %f", lowerBorder);
        increaseTrackingArea(sideBorder, upperBorder, lowerBorder);
    }
}

void TrackingArea::printTrackingArea() {
    printf("Tracking area is from [%f, %f, %f] to [%f, %f, %f], quadrat is of size %f, upper point is [%f, %f, %f], lower point is [%f, %f, %f].\n", a1.getV1(), a1.getV2(), a1.getV3(), a3.getV1(), a3.getV2(), a3.getV3(), a1.add(a2.mult(-1)).getLength(), up.getV1(), up.getV2(), up.getV3(), low.getV1(), low.getV2(), low.getV3());
}

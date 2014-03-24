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
#include "profiling.hpp"

using namespace std;

TrackingArea::TrackingArea(vector<Vector> cameraPosition, vector<Vector> cameraDirection, int numberCameras, double maxRange, Engine *ep) {
    this->maxRange = maxRange;
    this->cameraPosition = cameraPosition;
    this->cameraDirection = cameraDirection;
    this->numberCameras = numberCameras;
    this->ep = ep;
    setTrackingArea();
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
    Vector* calculateCenter(Engine *ep);
 *  checks whether a point x is in the TrackingArea or not
*/
bool TrackingArea::contains(Vector x) {
    if ((x.getV3() > up.getV3()) || (x.getV3() < low.getV3()) || (x.getV2() < a1.getV2()) || (x.getV2() > a2.getV2()) || (x.getV1() < a1.getV1()) || (x.getV1() > a3.getV1())) {
        return false;
    } else {

        double distX, distY, distZ, maxX, maxY, maxZ;
        if (x.getV3() > center.getV3()) {
            // x has to be in the upper pyramid
            Vector a = up;
            Vector u = Vector(1, 0, 0);
            Vector v = Vector(0, 1, 0);
            distZ = getDistPointPlane(a, u, v, x);

            u = Vector(1, 0, 0);
            v = Vector(0, 0, 1);
            distY = getDistPointPlane(a, u, v, x);

            u = Vector(0, 1, 0);
            v = Vector(0, 0, 1);
            distX = getDistPointPlane(a, u, v, x);

            maxX = center.getV1() - a1.getV1();
            maxY = center.getV2() - a1.getV2();
            maxZ = up.getV3() - center.getV3();

        } else {
            // x has to be in the lower pyramid
            Vector a = low;
            Vector u = Vector(1, 0, 0);
            Vector v = Vector(0, 1, 0);
            distZ = getDistPointPlane(a, u, v, x);

            u = Vector(1, 0, 0);
            v = Vector(0, 0, 1);
            distY = getDistPointPlane(a, u, v, x);

            u = Vector(0, 1, 0);
            v = Vector(0, 0, 1);
            distX = getDistPointPlane(a, u, v, x);

            maxX = center.getV1() - a1.getV1();
            maxY = center.getV2() - a1.getV2();
            maxZ = center.getV3() - low.getV3();
        }

        if ((maxX/maxZ < distX/distZ) || (maxY/maxZ < distY/distZ)) {
            return false;
        } else {
            return true;
        }
    }
}

bool TrackingArea::inTrackingArea(Vector cameraPosition, Vector cameraDirection, Vector x) {
    Matlab *m = new Matlab(ep);
    // center point of the floor of the pyramid
    Vector n = cameraPosition.add(cameraDirection.mult(maxRange/cameraDirection.getLength()));
    // finding direction vectors of the plane of the floor of the camera range pyramid.
    Vector u = Vector(cameraDirection.getV1(), cameraDirection.getV2(), -(cameraDirection.getV1()*cameraDirection.getV1() + cameraDirection.getV2()*cameraDirection.getV2())/cameraDirection.getV3());
    Vector v = cameraDirection.cross(u);

    // describing plane by line f and direction vector v
    Line *f = new Line(cameraDirection, u);

    // describing floor plane by a and b and cameraPosition
    Vector *a = new Vector(cameraDirection.getV1(), cameraDirection.getV2(), 0);
    Vector *b = new Vector(cameraDirection.getV1(), cameraDirection.getV2() + 0.1, 0);
    Line *g = new Line(cameraPosition, *a);

    Line horizontal = m->getIntersectionLineFastCalculation(*f, v, *g, *b);

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

bool TrackingArea::inCameraRange(Vector x) {
    int tracked = 0;
    for (int i = 0; i < numberCameras; i++) {
        if (inTrackingArea(cameraPosition[i], cameraDirection[i], x)) {
            tracked++;
        }
    }
    return (tracked > 2);
}

void TrackingArea::increaseTrackingArea(double posChange, double height, double heightPos, double heightNeg) {
    Vector center = getCenter();
    setA1(Vector(center.getV1() - posChange, center.getV2() - posChange, center.getV3() + height));
    setA2(Vector(center.getV1() - posChange, center.getV2() + posChange, center.getV3() + height));
    setA3(Vector(center.getV1() + posChange, center.getV2() + posChange, center.getV3() + height));
    setA4(Vector(center.getV1() + posChange, center.getV2() - posChange, center.getV3() + height));
    setUp(Vector(center.getV1(), center.getV2(), center.getV3() + heightPos));
    setLow(Vector(center.getV1(), center.getV2(), center.getV3() - heightNeg));
}

double TrackingArea::binarySearch(double leftBorder, double rightBorder, double posChange, double height, double heightPos, double heightNeg, int value) {

    double sideBorder = leftBorder;
    double middle = leftBorder + (rightBorder - leftBorder)/2;

    switch (value) {
    case 0: {
        increaseTrackingArea(middle, height, heightPos, heightNeg);
        // searching exact border of tracking area
        while (fabs(rightBorder - leftBorder) > 1) {
            ROS_DEBUG("binary search, side size: %.2f", 2 * middle);

            // checks whether all corners of tracking area are still tracked of all cameras
            if (inCameraRange(a1) && inCameraRange(a2) && inCameraRange(a3) && inCameraRange(a4)) {

                // border is between middle and rightBorder
                leftBorder = middle;
                sideBorder = middle;
            } else {
                // border is between leftBorder and middle
                rightBorder = middle;
            }
            middle = leftBorder + (rightBorder - leftBorder)/2;
            increaseTrackingArea(middle, height, heightPos, heightNeg);
        }

        ROS_DEBUG("binary search, side size: %.2f", 2 * middle);
        break;
    }
    case 1: {
        increaseTrackingArea(posChange, middle, heightPos, heightNeg);
        // searching exact border of tracking area
        while (fabs(rightBorder - leftBorder) > 1) {

            // checks whether all corners of tracking area are still tracked of all cameras

            if (inCameraRange(a1) && inCameraRange(a2) && inCameraRange(a3) && inCameraRange(a4)) {
                // border is between middle and rightBorder
                leftBorder = middle;
                sideBorder = middle;
            } else {
                // border is between leftBorder and middle
                rightBorder = middle;
            }
            middle = leftBorder + (rightBorder - leftBorder)/2;
            increaseTrackingArea(posChange, middle, heightPos, heightNeg);
            ROS_DEBUG("%.2f binary search, side size: %.2f", height, 2 * middle);
        }

        ROS_DEBUG("%.2f binary search, side size: %.2f", height, 2 * middle);
        break;
    }
    case 2: {
        increaseTrackingArea(posChange, height, middle, heightNeg);
        // searching exact border of tracking area
        while (fabs(rightBorder - leftBorder) > 1) {

            // checks whether all corners of tracking area are still tracked of all cameras
            if (inCameraRange(up)) {

                // border is between middle and rightBorder
                leftBorder = middle;
                sideBorder = middle;
            } else {
                // border is between leftBorder and middle
                rightBorder = middle;
            }
            middle = leftBorder + (rightBorder - leftBorder)/2;
            increaseTrackingArea(posChange, height, middle, heightNeg);
            ROS_DEBUG("upper binary search: %.2f", middle);
        }

        ROS_DEBUG("upper binary search: %.2f", middle);
        break;
    }
    case 3: {
        increaseTrackingArea(posChange, height, heightPos, middle);
        // searching exact border of tracking area
        while (fabs(rightBorder - leftBorder) > 1) {

            // checks whether all corners of tracking area are still tracked of all cameras
            if (inCameraRange(low)) {

                // border is between middle and rightBorder
                leftBorder = middle;
                sideBorder = middle;
            } else {
                // border is between leftBorder and middle
                rightBorder = middle;
            }
            middle = leftBorder + (rightBorder - leftBorder)/2;
            increaseTrackingArea(posChange, height, heightPos, middle);
            ROS_DEBUG("lower binary search: %.2f", middle);
        }

        ROS_DEBUG("lower binary search: %.2f", middle);
        break;
    }
    default:  {
        printf("Wrong value in binary search.");
        return NAN;
    }
    }
    return sideBorder;
}

// border is between basis + return and basis + return * 2
// value = 0, posChange, value = 1, height, value = 2, heightPos, value = 3, heightNeg
double TrackingArea::increaseSearch(double posChange, double height, double heightPos, double heightNeg, int value) {

    double diff = 0.5;

    increaseTrackingArea(posChange, height, heightPos, heightNeg);
    switch(value) {
    case 0: {
        while (inCameraRange(a1) && inCameraRange(a2) && inCameraRange(a3) && inCameraRange(a4)) {
            diff *= 2;
            increaseTrackingArea(posChange + diff, height, heightPos, heightNeg);
            ROS_DEBUG("increasing side size : %.2f", 2 * (posChange + diff));
        }
        break;
    }
    case 1: {
        while (inCameraRange(a1) && inCameraRange(a2) && inCameraRange(a3) && inCameraRange(a4)) {
            diff *= 2;
            increaseTrackingArea(posChange, height + diff, heightPos, heightNeg);
            ROS_DEBUG("increasing height: %.2f", (height + diff));
        }
        break;
    }
    case 2: {
        while (inCameraRange(up)) {
            diff *= 2;
            increaseTrackingArea(posChange, height, heightPos + diff, heightNeg);
            ROS_DEBUG("increasing heightPos: %.2f", diff + heightPos);
        }
        break;
    }
    case 3: {
        while (inCameraRange(low)) {
            diff *= 2;
            increaseTrackingArea(posChange, height, heightPos, heightNeg + diff);
            ROS_DEBUG("increasing heightNeg: %.2f", diff + heightNeg);
        }
        break;
    }
    default: {
        printf("wrong value in increasingSearch");
        return NAN;
    }
    }
    if (diff == 0.5) {
        return 0;
    } else {
        return diff/2;
    }
}

void TrackingArea::setTrackingArea() {
    Matlab *m = new Matlab(ep);
    Line *cameraLines = new Line[numberCameras];
    for (int i = 0; i < numberCameras; i++) {
        cameraLines[i] = Line();
        cameraLines[i] = Line(cameraPosition[i], cameraDirection[i]);
    }
    Vector center = m->interpolateLines(cameraLines, numberCameras);
    ROS_DEBUG("center is [%.2f, %.2f, %.2f]", center.getV1(), center.getV2(), center.getV3());
    if (!(inCameraRange(center))) {
        ROS_ERROR("center isn't tracked, maximal range is too small!");
        ROS_DEBUG("Maximal range is %f", maxRange);
        ROS_DEBUG("camera 0: [%f, %f, %f] + r * [%f, %f, %f]", cameraPosition[0].getV1(), cameraPosition[0].getV2(), cameraPosition[0].getV3(), cameraDirection[0].getV1(), cameraDirection[0].getV2(), cameraDirection[0].getV3());
        ROS_DEBUG("camera 0: [%f, %f, %f] + r * [%f, %f, %f]", cameraPosition[1].getV1(), cameraPosition[1].getV2(), cameraPosition[1].getV3(), cameraDirection[1].getV1(), cameraDirection[1].getV2(), cameraDirection[1].getV3());
        ROS_DEBUG("camera 0: [%f, %f, %f] + r * [%f, %f, %f]", cameraPosition[2].getV1(), cameraPosition[2].getV2(), cameraPosition[2].getV3(), cameraDirection[2].getV1(), cameraDirection[2].getV2(), cameraDirection[2].getV3());
        ROS_DEBUG("Distance of camera 0 to center is %f", center.add(cameraPosition[0].mult(-1)).getLength());
        ROS_DEBUG("Distance of camera 1 to center is %f", center.add(cameraPosition[1].mult(-1)).getLength());
        ROS_DEBUG("Distance of camera 2 to center is %f", center.add(cameraPosition[2].mult(-1)).getLength());
    } else {

        setCenter(center);
        setA1(center);
        setA2(center);
        setA3(center);
        setA4(center);
        setUp(center);
        setLow(center);

        /**
         * searching maximal size of square at height of center
         */

        ROS_INFO("Searching maximal square size at center height.");

        double maxCenterSize = increaseSearch(0, 0, 0, 0, 0);
        maxCenterSize = binarySearch(maxCenterSize, 2 * maxCenterSize, 0, 0, 0, 0, 0);
        ROS_DEBUG("maximal square size at center height is %.2f", maxCenterSize * 2);

        /**
          * searching optimal height (down) of square where the square size is biggest
          */

        ROS_INFO("Searching optimal height of square where the square size is biggest");

        double newSize = maxCenterSize;
        double oldSize = maxCenterSize;
        double heightLower = -4;
        // if lower = true, then the optimal middlepoint of trackingarea is lower than center
        bool lower = true;
        increaseTrackingArea(maxCenterSize, heightLower, 0, 0);
        if (!(inCameraRange(a1) && inCameraRange(a2) && inCameraRange(a3) && inCameraRange(a4))) {
            lower = false;
        }

        double leftBorderHeight, rightBorderHeight, middleHeight;
        if (lower) {

            ROS_INFO("Searching lower");

            do {
                oldSize = newSize;
                double diff = increaseSearch(newSize, heightLower, 0, 0, 0);
                newSize = binarySearch(newSize + diff, newSize + diff * 2, newSize, heightLower, 0, 0, 0);
                heightLower *= 2;
            } while (newSize > oldSize);

            // maximal width of tracking area is between heightLower and heightLower/2
            ROS_DEBUG("optimal width %.2f is between %.2f and %.2f", oldSize * 2, heightLower, heightLower/2);

            leftBorderHeight = heightLower/2;
            rightBorderHeight = heightLower;
            middleHeight = leftBorderHeight + (rightBorderHeight - leftBorderHeight)/2;

            // binary search while |rightBorderHeight - leftBorderHeight| > 1
            while (-rightBorderHeight + leftBorderHeight > 1) {
                double diff = increaseSearch(oldSize, middleHeight, 0, 0, 0);
                newSize = binarySearch(oldSize + diff, oldSize + diff * 2, 0, heightLower, 0, 0, 0);
                if (newSize > oldSize) {
                    leftBorderHeight = middleHeight;
                    oldSize = newSize;
                } else {
                    rightBorderHeight = middleHeight;
                }
                middleHeight = leftBorderHeight + (rightBorderHeight - leftBorderHeight)/2;
                ROS_DEBUG("lower %.2f, maximal quadrat size is %.2f", middleHeight, 2 * (oldSize));
            }
            ROS_DEBUG("Found optimal middlepoint, between %.2f and %.2f with size %.2f", rightBorderHeight,leftBorderHeight, 2 * oldSize);
        }

        /**
          * searching optimal height (up) of square where the square size ist biggest
          */
        else {

            ROS_INFO("Searching higher");

            // searching whether side size is bigger if height is lower
            double heightHigher = 4;
            newSize = maxCenterSize;
            oldSize = maxCenterSize;

            // decrease height while sideBorder gets bigger
            do {
                oldSize = newSize;
                double diff = increaseSearch(newSize, heightHigher, 0, 0, 0);
                newSize = binarySearch(newSize + diff, newSize + diff * 2, 0, heightHigher, 0, 0, 0);
                heightHigher *= 2;
            } while (newSize > oldSize);

            // maximal width of tracking area is between heightLower and heightLower/2
            ROS_DEBUG("maximal width %.2f is between %.2f and %.2f", oldSize * 2, heightHigher/2, heightHigher);

            leftBorderHeight = heightHigher/2;
            rightBorderHeight = heightHigher;
            middleHeight = leftBorderHeight + (rightBorderHeight - leftBorderHeight)/2;

            // binary search while |rightBorderHeight - leftBorderHeight| > 1
            while (rightBorderHeight - leftBorderHeight > 1) {
                double diff = increaseSearch(oldSize, middleHeight, 0, 0, 0);
                newSize = binarySearch(oldSize + diff, oldSize + diff * 2, 0, heightLower, 0, 0, 0);
                if (newSize > oldSize) {
                    leftBorderHeight = middleHeight;
                    oldSize = newSize;
                } else {
                    rightBorderHeight = middleHeight;
                }
                middleHeight = leftBorderHeight + (rightBorderHeight - leftBorderHeight)/2;
                ROS_DEBUG("upper %.2f, maximal quadrat size is %.2f", middleHeight, 2 * (oldSize));
            }
            ROS_DEBUG("Found optimal middlepoint, between %.2f and %.2f with size %.2f", leftBorderHeight, rightBorderHeight, 2* oldSize);
        }

        double maxWidth = oldSize;
        double optHeight = leftBorderHeight;
		
        /**
          * searching upper border of tracking area
          */

        ROS_INFO("Searching highest point.");

        double upperBorder = increaseSearch(0, 0, 0, 0, 2);
        upperBorder = binarySearch(upperBorder, 2 * upperBorder, 0, 0, 0, 0, 2);
        ROS_DEBUG("maximal upper size is %.2f", upperBorder);

        /**
          * searching lower border of tracking area
          */

        ROS_INFO("Searching lowest point.");

        double lowerBorder = increaseSearch(0, 0, 0, 0, 3);
        lowerBorder = binarySearch(lowerBorder, 2 * lowerBorder, 0, 0, 0, 0, 3);
        ROS_DEBUG("maximal lower size is %.2f", lowerBorder);

        // increase tracking area with calculated values
        increaseTrackingArea(maxWidth, optHeight, upperBorder, lowerBorder);
        ROS_DEBUG("Trackingarea has width %.2f and height %.2f", maxWidth * 2, lowerBorder + upperBorder);
    }
}

void TrackingArea::printTrackingArea() {
    ROS_DEBUG("Tracking area is from [%.2f, %.2f, %.2f] to [%.2f, %.2f, %.2f], square is of size %.2f, upper point is [%.2f, %.2f, %.2f], lower point is [%.2f, %.2f, %.2f].", a1.getV1(), a1.getV2(), a1.getV3(), a3.getV1(), a3.getV2(), a3.getV3(), a1.add(a2.mult(-1)).getLength(), up.getV1(), up.getV2(), up.getV3(), low.getV1(), low.getV2(), low.getV3());
}

Vector TrackingArea::getCenterOfTrackingArea() {
    Vector c = center;
    // center is center, just moved up/down
    c.setV3(a1.getV3());
    return c;
}

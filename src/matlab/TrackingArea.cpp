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

bool TrackingArea::inTrackingArea(Vector cameraPosition, Vector cameraDirection, double maxRange, Vector x, Engine *ep) {
    Matlab *m = new Matlab(ep);
    // center point of the floor of the pyramid1
    Vector n = cameraPosition.add(cameraDirection.mult(maxRange/cameraDirection.getLength()));
    // finding direction vectors of the plane of the floor of the camera range pyramid.
    Vector *u = new Vector(cameraDirection.getV1(), cameraDirection.getV2(), -(cameraDirection.getV1()*cameraDirection.getV1() + cameraDirection.getV2()*cameraDirection.getV2())/cameraDirection.getV3());
    Vector v = cameraDirection.cross(*u);

    // describing plane by line f and direction vector v
    Line *f = new Line(cameraDirection, *u);

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

bool TrackingArea::inCameraRange(std::vector<Vector> cameraPosition, std::vector<Vector> cameraDirection, int numberCameras, double maxRange, Vector x, Engine *ep) {
    //double startTime = getNanoTime();
    double notTracked = 0;
    for (int i = 0; i < numberCameras; i++) {
        if (inTrackingArea(cameraPosition[i], cameraDirection[i], maxRange, x, ep) == false) {
            notTracked++;
        }
    }
    /*double endTime = getNanoTime();
    ROS_DEBUG("Calculation was %f long", (endTime - startTime) / 1e9);*/
    if (notTracked < 2) {
        return true;
    } else {
        return false;
    }
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

void TrackingArea::setTrackingArea(std::vector<Vector> cameraPosition, std::vector<Vector> cameraDirection, int numberCameras, double maxRange, Engine *ep) {
    Matlab *m = new Matlab(ep);
    Line *cameraLines = new Line[numberCameras];
    for (int i = 0; i < numberCameras; i++) {
        cameraLines[i] = Line();
        cameraLines[i] = Line(cameraPosition[i], cameraDirection[i]);
    }
    Vector center = m->interpolateLines(cameraLines, numberCameras);
    ROS_DEBUG("center is [%.2f, %.2f, %.2f]", center.getV1(), center.getV2(), center.getV3());
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


        /**
         * searching maximal size of square at height of center
         */
        while (inCameraRange(cameraPosition, cameraDirection, numberCameras, maxRange, a1, ep) && inCameraRange(cameraPosition, cameraDirection, numberCameras, maxRange, a2, ep)
                && inCameraRange(cameraPosition, cameraDirection, numberCameras, maxRange, a3, ep) && inCameraRange(cameraPosition, cameraDirection, numberCameras, maxRange, a4, ep)) {
            posChange *= 2;
            increaseTrackingArea(posChange, 0, 0, 0);
            ROS_DEBUG("increasing, side size of center: %.2f", 2 * posChange);
        }

        // border is between leftBorder and rightBorder
        double leftBorder = posChange/2;
        double rightBorder = posChange;
        double middle = leftBorder + (rightBorder - leftBorder)/2;
        increaseTrackingArea(middle, 0, 0, 0);

        double sideBorder = leftBorder;
        // searching exact border of tracking area
        while (rightBorder - leftBorder > 1) {

            // checks whether all corners of tracking area are still tracked of all cameras
            if (inCameraRange(cameraPosition, cameraDirection, numberCameras, maxRange, a1, ep) && inCameraRange(cameraPosition, cameraDirection, numberCameras, maxRange, a2, ep)
                && inCameraRange(cameraPosition, cameraDirection, numberCameras, maxRange, a3, ep) && inCameraRange(cameraPosition, cameraDirection, numberCameras, maxRange, a4, ep)) {

                // border is between middle and rightBorder
                leftBorder = middle;
            } else {
                // border is between leftBorder and middle
                rightBorder = middle;
            }
            sideBorder = middle;
            middle = leftBorder + (rightBorder - leftBorder)/2;
            increaseTrackingArea(middle, 0, 0, 0);
            ROS_DEBUG("binary search, side size: %.2f", 2 * middle);
        }
        ROS_DEBUG("maximal square size is %.2f", sideBorder * 2);

        /**
          * searching optimal height (down) of square where the square size ist biggest
          */
        // searching whether side size is bigger if height is lower
        double heightLower = -1;
        double newSideBorder = sideBorder;
        // boolean that saves, whether size improves if going down.
        bool lower = false;

        // decrease height while sideBorder gets bigger
        do {
            // saves old side border in sideBorder
            sideBorder = newSideBorder;
            posChange = 0.5;
            increaseTrackingArea(sideBorder, heightLower, 0, 0);

            if (!(inCameraRange(cameraPosition, cameraDirection, numberCameras, maxRange, a1, ep) && inCameraRange(cameraPosition, cameraDirection, numberCameras, maxRange, a2, ep)
                    && inCameraRange(cameraPosition, cameraDirection, numberCameras, maxRange, a3, ep) && inCameraRange(cameraPosition, cameraDirection, numberCameras, maxRange, a4, ep))) {
                ROS_DEBUG("lower %.2f, not in range anymore", heightLower);
            } else {
                // searching new side border of tracking area
                while (inCameraRange(cameraPosition, cameraDirection, numberCameras, maxRange, a1, ep) && inCameraRange(cameraPosition, cameraDirection, numberCameras, maxRange, a2, ep)
                        && inCameraRange(cameraPosition, cameraDirection, numberCameras, maxRange, a3, ep) && inCameraRange(cameraPosition, cameraDirection, numberCameras, maxRange, a4, ep)) {
                    posChange *= 2;
                    increaseTrackingArea(sideBorder + posChange, heightLower, 0, 0);
                    // size does improve if going down
                    lower = true;
                    ROS_DEBUG("lower %.2f, increasing, side size: %.2f", heightLower, 2 * (posChange + sideBorder));
                }

                // new border is between leftBorder and rightBorder
                leftBorder = posChange/2 + sideBorder;
                rightBorder = posChange + sideBorder;
                middle = leftBorder + (rightBorder - leftBorder)/2;
                increaseTrackingArea(sideBorder + middle, heightLower, 0, 0);

                newSideBorder = leftBorder;
                // searching exact border of tracking area
                while (rightBorder - leftBorder > 1) {

                    // checks whether all corners of tracking area are still tracked of all cameras
                    if (inCameraRange(cameraPosition, cameraDirection, numberCameras, maxRange, a1, ep) && inCameraRange(cameraPosition, cameraDirection, numberCameras, maxRange, a2, ep)
                        && inCameraRange(cameraPosition, cameraDirection, numberCameras, maxRange, a3, ep) && inCameraRange(cameraPosition, cameraDirection, numberCameras, maxRange, a4, ep)) {

                        // border is between middle and rightBorder
                        leftBorder = middle;
                    } else {
                        // border is between leftBorder and middle
                        rightBorder = middle;
                    }
                    newSideBorder = middle;

                    middle = leftBorder + (rightBorder - leftBorder)/2;
                    increaseTrackingArea(middle, heightLower, 0, 0);
                    ROS_DEBUG("lower %.2f, binary search, side size: %f", heightLower, 2 * middle);
                }

                ROS_DEBUG("lower %.2f, maximal square size is %.2f", heightLower, 2 * (newSideBorder));
                heightLower *= 2;
            }
        } while (newSideBorder > sideBorder);

        double leftBorderHeight, rightBorderHeight, middleHeight;

        if (lower) {
            // maximal width of tracking area is between heightLower and heightLower/2
            ROS_DEBUG("maximal width %.2f is between %.2f and %.2f", sideBorder, heightLower, heightLower/2);


            leftBorderHeight = heightLower/2;
            rightBorderHeight = heightLower;
            middleHeight = leftBorderHeight - (-rightBorderHeight + leftBorderHeight)/2;
            newSideBorder = sideBorder;

            // binary search while |rightBorderHeight - leftBorderHeight| > 1
            while (-rightBorderHeight + leftBorderHeight > 1) {
                increaseTrackingArea(sideBorder, middleHeight, 0, 0);
                sideBorder = newSideBorder;
                posChange = 0.5;
                if (!(inCameraRange(cameraPosition, cameraDirection, numberCameras, maxRange, a1, ep) && inCameraRange(cameraPosition, cameraDirection, numberCameras, maxRange, a2, ep)
                        && inCameraRange(cameraPosition, cameraDirection, numberCameras, maxRange, a3, ep) && inCameraRange(cameraPosition, cameraDirection, numberCameras, maxRange, a4, ep))) {
                    ROS_DEBUG("%.2f is not in range anymore (%.2f)", middleHeight, sideBorder);
                    rightBorderHeight = middleHeight;
                    middleHeight = leftBorderHeight - (-rightBorderHeight + leftBorderHeight)/2;
                } else {
                    // searching new side border of tracking area
                    while (inCameraRange(cameraPosition, cameraDirection, numberCameras, maxRange, a1, ep) && inCameraRange(cameraPosition, cameraDirection, numberCameras, maxRange, a2, ep)
                            && inCameraRange(cameraPosition, cameraDirection, numberCameras, maxRange, a3, ep) && inCameraRange(cameraPosition, cameraDirection, numberCameras, maxRange, a4, ep)) {
                        posChange *= 2;
                        increaseTrackingArea(sideBorder + posChange, middleHeight, 0, 0);
                        ROS_DEBUG("lower %.2f, increasing, side size: %.2f", middleHeight, 2 * (posChange + sideBorder));
                    }

                    // new border is between leftBorder and rightBorder
                    leftBorder = posChange/2 + sideBorder;
                    rightBorder = posChange + sideBorder;
                    middle = leftBorder + (rightBorder - leftBorder)/2;
                    increaseTrackingArea(sideBorder + middle, middleHeight, 0, 0);

                    newSideBorder = leftBorder;
                    // searching exact border of tracking area
                    while (rightBorder - leftBorder > 1) {

                        // checks whether all corners of tracking area are still tracked of all cameras
                        if (inCameraRange(cameraPosition, cameraDirection, numberCameras, maxRange, a1, ep) && inCameraRange(cameraPosition, cameraDirection, numberCameras, maxRange, a2, ep)
                            && inCameraRange(cameraPosition, cameraDirection, numberCameras, maxRange, a3, ep) && inCameraRange(cameraPosition, cameraDirection, numberCameras, maxRange, a4, ep)) {

                            // border is between middle and rightBorder
                            leftBorder = middle;
                        } else {
                            // border is between leftBorder and middle
                            rightBorder = middle;
                        }

                        newSideBorder = middle;
                        middle = leftBorder + (rightBorder - leftBorder)/2;
                        increaseTrackingArea(middle, middleHeight, 0, 0);
                        ROS_DEBUG("lower %.2f, binary search, side size: %.2f", middleHeight, 2 * middle);
                    }
                    if (newSideBorder > sideBorder) {
                        leftBorderHeight = middleHeight;
                    } else {
                        rightBorderHeight = middleHeight;
                    }
                    middleHeight = leftBorderHeight + (-rightBorderHeight + leftBorderHeight)/2;
                    ROS_DEBUG("lower %.2f, maximal quadrat size is %.2f", middleHeight, 2 * (newSideBorder));
                }
            }
            ROS_DEBUG("Found optimal middlepoint, between %.2f and %.2f with size %.2f", leftBorderHeight, rightBorderHeight, sideBorder);
        }


        /**
          * searching optimal height (up) of square where the square size ist biggest
          */
        else {
            // searching whether side size is bigger if height is lower
            double heightHigher = 1;
            double newSideBorder = sideBorder;

            // decrease height while sideBorder gets bigger
            do {
                // saves old side border in sideBorder
                sideBorder = newSideBorder;
                posChange = 0.5;
                increaseTrackingArea(sideBorder, heightHigher, 0, 0);

                if (!(inCameraRange(cameraPosition, cameraDirection, numberCameras, maxRange, a1, ep) && inCameraRange(cameraPosition, cameraDirection, numberCameras, maxRange, a2, ep)
                        && inCameraRange(cameraPosition, cameraDirection, numberCameras, maxRange, a3, ep) && inCameraRange(cameraPosition, cameraDirection, numberCameras, maxRange, a4, ep))) {
                    ROS_DEBUG("upper %.2f, not in range anymore", heightHigher);
                } else {
                    // searching new side border of tracking area
                    while (inCameraRange(cameraPosition, cameraDirection, numberCameras, maxRange, a1, ep) && inCameraRange(cameraPosition, cameraDirection, numberCameras, maxRange, a2, ep)
                            && inCameraRange(cameraPosition, cameraDirection, numberCameras, maxRange, a3, ep) && inCameraRange(cameraPosition, cameraDirection, numberCameras, maxRange, a4, ep)) {
                        posChange *= 2;
                        increaseTrackingArea(sideBorder + posChange, heightHigher, 0, 0);
                        ROS_DEBUG("upper %.2f, increasing, side size: %.2f", heightHigher, 2 * (posChange + sideBorder));
                    }

                    // new border is between leftBorder and rightBorder
                    leftBorder = posChange/2 + sideBorder;
                    rightBorder = posChange + sideBorder;
                    middle = leftBorder + (rightBorder - leftBorder)/2;
                    increaseTrackingArea(sideBorder + middle, heightHigher, 0, 0);

                    newSideBorder = leftBorder;
                    // searching exact border of tracking area
                    while (rightBorder - leftBorder > 1) {

                        // checks whether all corners of tracking area are still tracked of all cameras
                        if (inCameraRange(cameraPosition, cameraDirection, numberCameras, maxRange, a1, ep) && inCameraRange(cameraPosition, cameraDirection, numberCameras, maxRange, a2, ep)
                            && inCameraRange(cameraPosition, cameraDirection, numberCameras, maxRange, a3, ep) && inCameraRange(cameraPosition, cameraDirection, numberCameras, maxRange, a4, ep)) {

                            // border is between middle and rightBorder
                            leftBorder = middle;
                        } else {
                            // border is between leftBorder and middle
                            rightBorder = middle;
                        }
                        newSideBorder = middle;

                        middle = leftBorder + (rightBorder - leftBorder)/2;
                        increaseTrackingArea(middle, heightHigher, 0, 0);
                        ROS_DEBUG("upper %.2f, binary search, side size: %.2f", heightHigher, 2 * middle);
                    }

                    ROS_DEBUG("upper %.2f, maximal square size is %.2f", heightHigher, 2 * (newSideBorder));
                    heightHigher *= 2;
                }
            } while (newSideBorder > sideBorder);

            // maximal width of tracking area is between heightLower and heightLower/2
            ROS_DEBUG("maximal width %.2f is between %.2f and %.2f", sideBorder, heightHigher/2, heightHigher);


            leftBorderHeight = heightHigher/2;
            rightBorderHeight = heightHigher;
            middleHeight = leftBorderHeight + (rightBorderHeight - leftBorderHeight)/2;
            newSideBorder = sideBorder;

            // binary search while |rightBorderHeight - leftBorderHeight| > 1
            while (rightBorderHeight - leftBorderHeight > 1) {
                increaseTrackingArea(sideBorder, middleHeight, 0, 0);
                sideBorder = newSideBorder;
                posChange = 0.5;
                if (!(inCameraRange(cameraPosition, cameraDirection, numberCameras, maxRange, a1, ep) && inCameraRange(cameraPosition, cameraDirection, numberCameras, maxRange, a2, ep)
                        && inCameraRange(cameraPosition, cameraDirection, numberCameras, maxRange, a3, ep) && inCameraRange(cameraPosition, cameraDirection, numberCameras, maxRange, a4, ep))) {
                    ROS_DEBUG("%.2f is not in range anymore (%.2f)", middleHeight, sideBorder);
                    rightBorderHeight = middleHeight;
                    middleHeight = leftBorderHeight + (rightBorderHeight - leftBorderHeight)/2;
                } else {
                    // searching new side border of tracking area
                    while (inCameraRange(cameraPosition, cameraDirection, numberCameras, maxRange, a1, ep) && inCameraRange(cameraPosition, cameraDirection, numberCameras, maxRange, a2, ep)
                            && inCameraRange(cameraPosition, cameraDirection, numberCameras, maxRange, a3, ep) && inCameraRange(cameraPosition, cameraDirection, numberCameras, maxRange, a4, ep)) {
                        posChange *= 2;
                        increaseTrackingArea(sideBorder + posChange, middleHeight, 0, 0);
                        ROS_DEBUG("upper %.2f, increasing, side size: %.2f", middleHeight, 2 * (posChange + sideBorder));
                    }

                    // new border is between leftBorder and rightBorder
                    leftBorder = posChange/2 + sideBorder;
                    rightBorder = posChange + sideBorder;
                    middle = leftBorder + (rightBorder - leftBorder)/2;
                    increaseTrackingArea(sideBorder + middle, middleHeight, 0, 0);

                    newSideBorder = leftBorder;
                    // searching exact border of tracking area
                    while (rightBorder - leftBorder > 1) {

                        // checks whether all corners of tracking area are still tracked of all cameras
                        if (inCameraRange(cameraPosition, cameraDirection, numberCameras, maxRange, a1, ep) && inCameraRange(cameraPosition, cameraDirection, numberCameras, maxRange, a2, ep)
                            && inCameraRange(cameraPosition, cameraDirection, numberCameras, maxRange, a3, ep) && inCameraRange(cameraPosition, cameraDirection, numberCameras, maxRange, a4, ep)) {

                            // border is between middle and rightBorder
                            leftBorder = middle;
                        } else {
                            // border is between leftBorder and middle
                            rightBorder = middle;
                        }

                        newSideBorder = middle;
                        middle = leftBorder + (rightBorder - leftBorder)/2;
                        increaseTrackingArea(middle, middleHeight, 0, 0);
                        ROS_DEBUG("upper %.2f, binary search, side size: %.2f", middleHeight, 2 * middle);
                    }
                    if (newSideBorder > sideBorder) {
                        leftBorderHeight = middleHeight;
                    } else {
                        rightBorderHeight = middleHeight;
                    }
                    middleHeight = leftBorderHeight + (rightBorderHeight - leftBorderHeight)/2;
                    ROS_DEBUG("upper %.2f, maximal quadrat size is %.2f", middleHeight, 2 * (newSideBorder));
                }
            }
            ROS_DEBUG("Found optimal middlepoint, between %.2f and %.2f with size %.2f", leftBorderHeight, rightBorderHeight, 2* (sideBorder));
        }

        double maxWidth = sideBorder;
        double maxLower = leftBorderHeight;
		
		
        /**
          * searching upper border of tracking area
          */

        posChange = 1;
        while (inCameraRange(cameraPosition, cameraDirection, numberCameras, maxRange, up, ep)) {
            posChange *= 2;
            increaseTrackingArea(0, 0, posChange, 0);
            ROS_DEBUG("increasing, upper size of center: %.2f", posChange);
        }

        // border is between leftBorder and rightBorder
        leftBorder = posChange/2;
        rightBorder = posChange;
        middle = leftBorder + (rightBorder - leftBorder)/2;
        increaseTrackingArea(0, 0, middle, 0);

        double upperBorder = leftBorder;
        // searching exact border of tracking area
        while (rightBorder - leftBorder > 1) {

            // checks whether all corners of tracking area are still tracked of all cameras
            if (inCameraRange(cameraPosition, cameraDirection, numberCameras, maxRange, up, ep)) {

                // border is between middle and rightBorder
                leftBorder = middle;
            } else {
                // border is between leftBorder and middle
                rightBorder = middle;
            }
            upperBorder = middle;
            middle = leftBorder + (rightBorder - leftBorder)/2;
            increaseTrackingArea(0, 0, middle, 0);
            ROS_DEBUG("binary search, upper size: %.2f", middle);
        }

        ROS_DEBUG("maximal upper size is %.2f", upperBorder);

        /**
          * searching lower border of tracking area
          */
        posChange = 1;
        while (inCameraRange(cameraPosition, cameraDirection, numberCameras, maxRange, low, ep)) {
            posChange *= 2;
            increaseTrackingArea(0, 0, 0, posChange);
            ROS_DEBUG("increasing, lower size of center: %.2f", posChange);
        }

        // border is between leftBorder and rightBorder
        leftBorder = posChange/2;
        rightBorder = posChange;
        middle = leftBorder + (rightBorder - leftBorder)/2;
        increaseTrackingArea(0, 0, 0, middle);

        double lowerBorder = 0;
        // searching exact border of tracking area
        while (rightBorder - leftBorder > 1) {

            // checks whether all corners of tracking area are still tracked of all cameras
            if (inCameraRange(cameraPosition, cameraDirection, numberCameras, maxRange, low, ep)) {

                // border is between middle and rightBorder
                leftBorder = middle;
            } else {
                // border is between leftBorder and middle
                rightBorder = middle;
            }
            lowerBorder = middle;
            middle = leftBorder + (rightBorder - leftBorder)/2;
            increaseTrackingArea(0, 0, 0, middle);
            ROS_DEBUG("binary search, lower size: %.2f", middle);
        }

        ROS_DEBUG("maximal lower size is %.2f", lowerBorder);

        // increase tracking area with calculated values
        increaseTrackingArea(maxWidth, maxLower, upperBorder, lowerBorder);
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

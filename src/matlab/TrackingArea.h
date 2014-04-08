/**
 * TrackingArea.h
 * tracking area is in form of two pyramides, one upside down, one normal. The ground plain of the two pyramides is the same.
 *
 *  Created on: 18.01.2014
 *      Author: daniela
 */

#ifndef TRACKINGAREA_H_
#define TRACKINGAREA_H_

#include "Vector.h"
#include <vector>

class TrackingArea {
private:
    /**
     * a1, a2, a3, a4 are the corners of the square in the middle, maximal size of quadrat.
     */
    Vector a1, a2, a3, a4;

    /**
     * center is perpendicular foot point of all orientation lines of cameras, low is the lowest point of the tracking area, up is the highest point of the tracking area.
     */
    Vector center, low, up;


    /**
     * cameraPosition: positions of cameras
     */
    std::vector<Vector> cameraPosition;

    /**
     * cameraDirection: orientation of cameras
     */
    std::vector<Vector> cameraDirection;

    /**
     * maximal range of camera
     */
    double maxRange;

    /**
     * number of cameras that track the tracking area
     */
    int numberCameras;

    /**
     * increases tracking area as middle point center, values are the difference to the values of center point.
     * @param posChange +- x-values and y-values of a1 to a4
     * @param height center.V3() + height is z value of a1 to a4
     * @param heightPos up has x-/y-values of center and z-value is center.V3() + heightPos
     * @param heightNeg low has x-/y-values of center and z-value is center.V3() - heightNeg
     */
    void increaseTrackingArea(double posChange, double height, double heightPos, double heightNeg);


    /**
     * getter.
     * @return center in average nearest point to all camera lines of position + r*orientation. Not the center of trackingarea.
     */
    Vector getCenter();

    /**
     * @brief setter.
     * @param center in average nearest point to all camera lines of position + r*orientation. Not the center of trackingarea.
     */
    void setCenter(Vector center);

    /**
     * checks whether a point can be tracked of a single camera
     * @param cameraPosition position of camera
     * @param cameraDirection orientation of camera
     * @param x point that should be checked
     * @return true if x can be tracked
     */
    bool inTrackingArea(Vector cameraPosition, Vector cameraDirection, Vector x);

public:
    /**
     * constructor.
     * @param cameraPosition vector that contains camera positions of cameras
     * @param cameraDirection vector that contains camera directions of cameras
     * @param numberCameras number of cameras
     * @param maxRange maximal range of cameras
     */
    TrackingArea(std::vector<Vector> cameraPosition, std::vector<Vector>  cameraDirection, int numberCameras, double maxRange);

    /**
     * constructor.
     */
    TrackingArea() {}

    /**
     * getter.
     * @return one corner of square of tracking area
     */
    Vector getA1();

    /**
     * getter.
     * @return one corner of square of tracking area
     */
    Vector getA2();

    /**
     * getter.
     * @return one corner of square of tracking area
     */
    Vector getA3();

    /**
     * getter.
     * @return one corner of square of tracking area
     */
    Vector getA4();

    /**
     * getter.
     * @return lowest point of tracking area
     */
    Vector getLow();

    /**
     * getter.
     * @return highest point of tracking area
     */
    Vector getUp();

    /**
     * setter.
     * @param a1 one corner of square of tracking area
     */
	void setA1(Vector a1);

    /**
     * setter.
     * @param a2 one corner of square of tracking area
     */
	void setA2(Vector a2);

    /**
     * setter.
     * @param a3 one corner of square of tracking area
     */
	void setA3(Vector a3);

    /**
     * setter.
     * @param a4 one corner of square of tracking area
     */
	void setA4(Vector a4);

    /**
     * setter.
     * @param low lowest point of tracking area
     */
    void setLow(Vector low);

    /**
     * @brief setter.
     * @param up highest point of tracking area
     */
    void setUp(Vector up);

    /**
     * checks whether a vector is in calculated tracking area or not.
     * @param x vector that should be checked
     * @return whether x is in calculated tracking area or not
     */
    bool contains(Vector x);

    /**
     * sets/calculates the tracking area with binary search
     */
    void setTrackingArea();

    /**
     * prints two opposite placed points of the square and lowest and highest point
     */
    void printTrackingArea();

    /**
     * getter.
     * @return center of tracking area
     */
    Vector getCenterOfTrackingArea();

    /**
     * searches border thorugh multiplication of 2
     * @param posChange difference of x-/y-values to center of a1 to a4
     * @param height difference of z-values to center of a1 to a4
     * @param heightPos difference of z-value of up to center z-value
     * @param heightNeg difference of z-value of low to center z-value
     * @param value 0: posChange, 1: height, 2: heightPos, 3: heightNeg increasing
     * @return
     */
    double increaseSearch(double posChange, double height, double heightPos, double heightNeg, int value);

    /**
     * binary search
     * @param leftBorder left border of binary search
     * @param rightBorder right border of binary search
     * @param posChange difference of x-/y-values to center of a1 to a4
     * @param height difference of z-values to center of a1 to a4
     * @param heightPos difference of z-value of up to center z-value
     * @param heightNeg difference of z-value of low to center z-value
     * @param value 0: posChange, 1: height, 2: heightPos, 3: heightNeg binary searching
     * @return
     */
    double binarySearch(double leftBorder, double rightBorder, double posChange, double height, double heightPos, double heightNeg, int value);

    /**
     * checks whether a point can be tracked of at least 2 cameras
     * @param x point
     * @return true if at least 2 cameras can track x
     */
    bool inCameraRange(Vector x);

};

#endif /* TRACKINGAREA_H_ */

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
     * moves form along the z-axis
     * @param height is difference, that the tracking area is moved from the center z-value.
     */
    void increaseTrackingArea(double posChange, double height);

    /**
     * creates a tracking area in form of a, that has center as center and whose quader height is 2* height and whose length width is 2*posChange
     */
    void increaseTrackingArea(double posChange, double heightPos, double heightNeg);


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

public:
    TrackingArea(std::vector<Vector> cameraPosition, std::vector<Vector>  cameraDirection, int numberCameras, double maxRange, Engine *ep);
    TrackingArea() {};
    Vector getA1();
    Vector getA2();
    Vector getA3();
    Vector getA4();
    Vector getLow();
    Vector getUp();

    /**
     * setter.
     * @param a1 one corner of square
     */
	void setA1(Vector a1);

    /**
     * setter.
     * @param a2 one corner of square
     */
	void setA2(Vector a2);

    /**
     * setter.
     * @param a3 one corner of square
     */
	void setA3(Vector a3);

    /**
     * setter.
     * @param a4 one corner of square
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

	double getHeight();
	double getWidth();
	double getLength();
	bool contains(Vector x);
    bool inTrackingArea(Vector cameraPosition, Vector cameraDirection, double maxRange, Vector x, Engine *ep);
	double getDistPointPlane(Vector a1, Vector u, Vector v, Vector x);
	Vector getPerpPointPlane(Vector a, Vector u, Vector v, Vector x);
    bool inCameraRange(std::vector<Vector> cameraPosition, std::vector<Vector> cameraDirection, int numberCameras, double maxRange, Vector x, Engine *ep);
    void setTrackingArea(std::vector<Vector> cameraPosition, std::vector<Vector> cameraDirection, int numberCameras, double maxRange, Engine *ep);

    /**
     * prints two opposite placed points of the square and lowest and highest point
     */
    void printTrackingArea();
};

#endif /* TRACKINGAREA_H_ */

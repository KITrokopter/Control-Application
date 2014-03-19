/*
 * TrackingArea.h
 *
 *  Created on: 18.01.2014
 *      Author: daniela
 */

#ifndef TRACKINGAREA_H_
#define TRACKINGAREA_H_

#include "Vector.h"
#include <vector>

class TrackingArea {
// a1 to a4 is clockwisely one plain, b1 to b4 the same, ai is below of bi
private:
    Vector a1, a2, a3, a4, low, up, center;

    /**
     * moves form along the z-axis
     * @param height is difference, that the tracking area is moved from the center z-value.
     */
    void increaseTrackingArea(double height);

    /**
     * creates a tracking area in form of a quader, that has center as center and whose quader height is 2* height and whose length width is 2*posChange

     */
    void increaseTrackingArea(double posChange, double heightPos, double heightNeg);

public:
    TrackingArea(Vector a1, Vector a2, Vector a3, Vector a4, Vector low, Vector up);
	TrackingArea(std::vector<Vector> cameraPosition, std::vector<Vector>  cameraDirection, int numberCameras, double maxRange, Engine *ep); 
    TrackingArea() {};
    Vector getA1();
    Vector getA2();
    Vector getA3();
    Vector getA4();
    Vector getLow();
    Vector getUp();
	void setA1(Vector a1);
	void setA2(Vector a2);
	void setA3(Vector a3);
	void setA4(Vector a4);
    void setLow(Vector low);
    void setUp(Vector up);
	Vector getCenter();
	void setCenter(Vector center);
	Vector* calculateCenter(Engine *ep);
	double getHeight();
	double getWidth();
	double getLength();
	bool contains(Vector x);
    bool inTrackingArea(Vector cameraPosition, Vector cameraDirection, double maxRange, Vector x, Engine *ep);
	double getDistPointPlane(Vector a1, Vector u, Vector v, Vector x);
	Vector getPerpPointPlane(Vector a, Vector u, Vector v, Vector x);
    //double getVolume(double height);
    bool inCameraRange(std::vector<Vector> cameraPosition, std::vector<Vector> cameraDirection, int numberCameras, double maxRange, Vector x, Engine *ep);
    void setTrackingArea(std::vector<Vector> cameraPosition, std::vector<Vector> cameraDirection, int numberCameras, double maxRange, Engine *ep);
	void printTrackingArea();
};

#endif /* TRACKINGAREA_H_ */

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
private: Vector a1, a2, a3, a4, b1, b2, b3, b4, center;
public:
	TrackingArea(Vector a1, Vector a2, Vector a3, Vector a4, Vector b1, Vector b2, Vector b3, Vector b4);
	TrackingArea(std::vector<Vector> cameraPosition, std::vector<Vector>  cameraDirection, int numberCameras, double maxRange, Engine *ep); 
	TrackingArea(){};
    Vector getA1();
    Vector getA2();
    Vector getA3();
    Vector getA4();
    Vector getB1();
    Vector getB2();
    Vector getB3();
    Vector getB4();
	void setA1(Vector a1);
	void setA2(Vector a2);
	void setA3(Vector a3);
	void setA4(Vector a4);
	void setB1(Vector b1);
	void setB2(Vector b2);
	void setB3(Vector b3);
	void setB4(Vector b4);
	Vector getCenter();
	void setCenter(Vector center);
	Vector* calculateCenter(Engine *ep);
	double getHeight();
	double getWidth();
	double getLength();
	bool contains(Vector x);
    bool inTrackingArea(Vector cameraPosition, Vector cameraDirection, double maxRange, Vector x, Engine *ep);
	double getVectorLength(Vector a, Vector b);
	double getDistPointPlane(Vector a1, Vector u, Vector v, Vector x);
	Vector getPerpPointPlane(Vector a, Vector u, Vector v, Vector x);
    bool inCameraRange(std::vector<Vector> cameraPosition, std::vector<Vector> cameraDirection, int numberCameras, double maxRange, Vector x, Engine *ep);
    void setTrackingArea(std::vector<Vector> cameraPosition, std::vector<Vector> cameraDirection, int numberCameras, double maxRange, Engine *ep);
	void printTrackingArea();
};

#endif /* TRACKINGAREA_H_ */

/*
 * AmccCalibration.h
 *
 *  Created on: 13.01.2014
 *      Author: daniela
 */

#ifndef AmccCalibration_H_
#define AmccCalibration_H_
#include "Line.h"
#include "Vector.h"
#include "engine.h"

class AmccCalibration {
private: Engine *ep;
public:
	AmccCalibration();
	AmccCalibration(Engine *ep);
	// sqareLengthX: length of each square of the checkboard in X direction (mm)
	// number of square corners of the checkerboard in the X direction
	void multiCameraCalibration(int numberCameras, double squareLengthX, double squareLengthY, int numberSquareCornersX, int numberSquareCornersY);
};

#endif /* AmccCalibration_H_ */

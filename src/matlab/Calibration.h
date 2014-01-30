/*
 * Calibration.h
 *
 *  Created on: 13.01.2014
 *      Author: daniela
 */

#ifndef Calibration_H_
#define Calibration_H_
#include "Line.h"
#include "Vector.h"
#include "engine.h"

class Calibration {
private: Engine *ep;
public:
	Calibration();
	Calibration(Engine *ep);
	// sqareLengthX: length of each square of the checkboard in X direction (mm)
	// number of square corners of the checkerboard in the X direction
	void multiCameraCalibration(int numberCameras, double squareLengthX, double squareLengthY, int numberSquareCornersX, int numberSquareCornersY);
};

#endif /* Calibration_H_ */

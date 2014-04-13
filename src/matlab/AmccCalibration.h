/*
 * AmccCalibration.h
 *
 * multi camera calibration with amcc toolbox
 * used method of
 * M. Warren, D. McKinnon, B. Upcroft, "Online Calibration of Stereo Rigs for Long-Term Autonomy", in International Conference on Robotics and Automation, Karlsruhe, Germany, 2013.
 *
 *  Created on: 13.01.2014
 *      Author: daniela
 */

#ifndef AmccCalibration_H_
#define AmccCalibration_H_
#include "Line.h"
#include "Vector.h"
#include <engine.h>

class AmccCalibration {
private: Engine *ep;
public:

    /**
     * @brief AmccCalibration creates a new Matlab engine
     */
    AmccCalibration();

    /**
     * @brief AmccCalibration
     * @param ep matlab engine
     */
    AmccCalibration(Engine *ep);

    /**
     * @brief multiCameraCalibration: opens amcc toolbox CALLBACK
     * M. Warren, D. McKinnon, B. Upcroft, "Online Calibration of Stereo Rigs for Long-Term Autonomy", in International Conference on Robotics and Automation, Karlsruhe, Germany, 2013.
     * @param numberCameras: number cameras that should be calibrated
     * @param squareLengthX: length of each square of the checkboard in X direction (mm)
     * @param squareLengthY: length of each square of the checkboard in Y direction (mm)
     * @param numberSquareCornersX: number of square corners of the checkboard in the X direction
     * @param numberSquareCornersY: number of square corners of the checkboard in the Y direction
     */
	void multiCameraCalibration(int numberCameras, double squareLengthX, double squareLengthY, int numberSquareCornersX, int numberSquareCornersY);
};

#endif /* AmccCalibration_H_ */

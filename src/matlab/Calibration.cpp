/*
 * Matlab.cpp
 *
 *  Created on: 13.01.2014
 *      Author: daniela
 */

#include "Matlab.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <iostream>
#include "engine.h"
#include "Vector.h"
#include "Line.h"
#include "Calibration.h"
#define  BUFSIZE 256


Calibration::Calibration() {

}

void Calibration::multiCameraCalibration(double numberCameras, double squareLengthX, double squareLengthY, double numberSquareCornersX, double numberSquareCornersY, Engine *ep) {
	mxArray *slx, *sly, *nscx, *nscy, *nc;

	double dataSlx[1] = {squareLengthX};
	slx = mxCreateDoubleMatrix(1, 1, mxREAL);
	memcpy((void *)mxGetPr(slx), (void *)dataSlx, sizeof(dataSlx));
	engPutVariable(ep, "dX", slx);

	double dataSly[1] = {squareLengthY};
	sly = mxCreateDoubleMatrix(1, 1, mxREAL);
	memcpy((void *)mxGetPr(sly), (void *)dataSly, sizeof(dataSly));
	engPutVariable(ep, "dY", sly);
	
	double dataNscx[1] = {numberSquareCornersX};
	nscx = mxCreateDoubleMatrix(1, 1, mxREAL);
	memcpy((void *)mxGetPr(nscx), (void *)dataNscx, sizeof(dataNscx));
	engPutVariable(ep, "nx_crnrs", nscx);

	double dataNscy[1] = {numberSquareCornersY};
	nscy = mxCreateDoubleMatrix(1, 1, mxREAL);
	memcpy((void *)mxGetPr(nscy), (void *)dataNscy, sizeof(dataNscy));
	engPutVariable(ep, "ny_crnrs", nscy);

	double dataNc[1] = {numberCameras};
	nc = mxCreateDoubleMatrix(1, 1, mxREAL);
	memcpy((void *)mxGetPr(nc), (void *)dataNc, sizeof(dataNc));
	engPutVariable(ep, "nc", nc);

	// Where the images are (forward slashes only, and must include a trailing slash)
    engEvalString(ep, "input_dir = '/home/dani/input_calibrationtest/';");

	// Where the data will be saved (forward slashes only, and must include a trailing slash). This folder should already exist
    engEvalString(ep, "output_dir = '/home/dani/output_calibrationtest/';");

	// Image format: jpeg, bmp, tiff, png etc.
    engEvalString(ep, "format_image = 'jpeg'");

	// tolerance in pixels of reprojection of checkerboard corners
	engEvalString(ep, "proj_tol = 0.6;");

	// The index of the cameras to calibrate. In this example we are calibrating four cameras with sequential naming.
	// camera_vec = [0 1 2 3]; % version 1.2 and before
	// camera_vec = [0 1; 0 2; 0 3]';
    engEvalString(ep, "camera_vec = [zeros(1,(nc-1)); (1:(nc-1))]");

	// The index of the cameras to be rotated (1 for rotating 180 degrees)
	// rotcam = [0 0 0 0]; % version 1.2 and before
    engEvalString(ep, "rotcam = zeros(2, (nc))';");

	// indicate whether or not to use the fisheye calibration routine (not strictly required).
	engEvalString(ep, "fisheye = false;");

	// indicate whether or not to use the third radial distortion term when doing a projective calibration (not strictly required)
	engEvalString(ep, "k3_enable = false;");

	// the base naming convention for the calibration images (not strictly required), will default to the 'camX_image' convention if not used.
	// cam_names = ['cam0_image', 'cam1_image', 'cam2_image', 'cam3_image']; % version 1.2 and before
	engEvalString(ep, "cam_names = ['cam0_image'; 'cam1_image'; 'cam2_image'; 'cam3_image'];");

	// indicate whether or not to use the batch mode of the stereo calibrator (not strictly required)
	engEvalString(ep, "batch = false;");

	// Perform the calibration

	engEvalString(ep, "auto_multi_calibrator_efficient(camera_vec, input_dir, output_dir, format_image, dX, dY, nx_crnrs, ny_crnrs, proj_tol, rotcam, cam_names, fisheye, k3_enable, batch);");
}

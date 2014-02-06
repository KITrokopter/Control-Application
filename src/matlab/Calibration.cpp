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
    Engine *ep;
    // starts a MATLAB process
    if (!(ep = engOpen(""))) {
            fprintf(stderr, "\nCan't start MATLAB engine\n");
    } else {
        this->ep = ep;
    }
}

Calibration::Calibration(Engine *ep) {
    this->ep = ep;
}

//M. Warren, D. McKinnon, B. Upcroft, "Online Calibration of Stereo Rigs for Long-Term Autonomy", in International Conference on Robotics and Automation, Karlsruhe, Germany, 2013.
void Calibration::multiCameraCalibration(int numberCameras, double squareLengthX, double squareLengthY, int numberSquareCornersX, int numberSquareCornersY) {
    mxArray *slx, *sly, *nscx, *nscy, *nc;

    int dataSlx[1] = {squareLengthX};
    slx = mxCreateDoubleMatrix(1, 1, mxREAL);
    memcpy((void *)mxGetPr(slx), (void *)dataSlx, sizeof(dataSlx));
    engPutVariable(ep, "dX", slx);

    int dataSly[1] = {squareLengthY};
    sly = mxCreateDoubleMatrix(1, 1, mxREAL);
    memcpy((void *)mxGetPr(sly), (void *)dataSly, sizeof(dataSly));
    engPutVariable(ep, "dY", sly);

    int dataNscx[1] = {numberSquareCornersX};
    nscx = mxCreateNumericMatrix(1, 1, mxSINGLE_CLASS, mxREAL);
    memcpy((void *)mxGetPr(nscx), (void *)dataNscx, sizeof(dataNscx));
    engPutVariable(ep, "nx_crnrs", nscx);

    int dataNscy[1] = {numberSquareCornersY};
    nscy = mxCreateNumericMatrix(1, 1, mxSINGLE_CLASS, mxREAL);
    memcpy((void *)mxGetPr(nscy), (void *)dataNscy, sizeof(dataNscy));
    engPutVariable(ep, "ny_crnrs", nscy);

    int dataNc[1] = {numberCameras};
    nc = mxCreateNumericMatrix(1, 1, mxINT32_CLASS, mxREAL);
    memcpy((void *)mxGetPr(nc), (void *)dataNc, sizeof(dataNc));
    engPutVariable(ep, "nc", nc);

    // Where the images are (forward slashes only, and must include a trailing slash)
    engEvalString(ep, "input_dir = '/home/dani/input_calibrationtest/';");

    // Where the data will be saved (forward slashes only, and must include a trailing slash). This folder should already exist
    engEvalString(ep, "output_dir = '/home/dani/output_calibrationtest/';");

    // Image format: jpeg, bmp, tiff, png etc.
    engEvalString(ep, "format_image = 'jpg'");

    // tolerance in pixels of reprojection of checkerboard corners
    engEvalString(ep, "proj_tol = 0.6;");

    // The index of the cameras to calibrate. In this example we are calibrating four cameras with sequential naming.
    // camera_vec = [0 1 2 3]; % version 1.2 and before
    // camera_vec = [0 1; 0 2; 0 3]';
    engEvalString(ep, "camera_vec = [zeros(1,(nc), 'single'); (1:(cast(nc, 'single')))]");

    // The index of the cameras to be rotated (1 for rotating 180 degrees)
    // rotcam = [0 0 0 0]; % version 1.2 and before
    engEvalString(ep, "rotcam = zeros(2, (nc), 'single')';");

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

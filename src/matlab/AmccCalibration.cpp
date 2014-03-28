/*
 * AmccCalibration.cpp
 *
 * Used method of:
 * M. Warren, D. McKinnon, B. Upcroft, "Online Calibration of Stereo Rigs for Long-Term Autonomy", in International Conference on Robotics and Automation, Karlsruhe, Germany, 2013.
 *
 *  Created on: 13.01.2014
 *      Author: daniela
 */

#include "Matlab.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <sstream>
#include <cstring>
#include "AmccCalibration.h"
#include <ros/ros.h>


AmccCalibration::AmccCalibration() {
    Engine *ep;
    // starts a MATLAB process
    if (!(ep = engOpen(""))) {
            fprintf(stderr, "\nCan't start MATLAB engine\n");
    } else {
        this->ep = ep;
    }
}

AmccCalibration::AmccCalibration(Engine *ep) {
    this->ep = ep;
}

//M. Warren, D. McKinnon, B. Upcroft, "Online Calibration of Stereo Rigs for Long-Term Autonomy", in International Conference on Robotics and Automation, Karlsruhe, Germany, 2013.
void AmccCalibration::multiCameraCalibration(int numberCameras, double squareLengthX, double squareLengthY, int numberSquareCornersX, int numberSquareCornersY) {
    mxArray *slx, *sly, *nscx, *nscy, *nc;

    // for debuging saves Matlab log in input folder
    engEvalString(ep, "diary('/tmp/calibrationImages/log');");

    // input of parameters
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
    engEvalString(ep, "input_dir = '/tmp/calibrationImages/';");

    // Where the data will be saved (forward slashes only, and must include a trailing slash). This folder should already exist
    engEvalString(ep, "output_dir = '/tmp/calibrationResult/';");

    // Image format: jpeg, bmp, tiff, png etc.
    engEvalString(ep, "format_image = 'png'");

    // tolerance in pixels of reprojection of checkerboard corners
    engEvalString(ep, "proj_tol = 2.0;");

    // The index of the cameras to calibrate. In this example we are calibrating four cameras with sequential naming.
    // camera_vec = [0 1; 0 2; 0 3]';
    engEvalString(ep, "camera_vec = [zeros(1,(nc - 1), 'single'); (1:(cast(nc - 1, 'single')))]");
   // engEvalString(ep, "camera_control_vec = [0, nc]");
   // engEvalString(ep, "camera_vec = [camera_vec,  vec]");

    // The index of the cameras to be rotated (1 for rotating 180 degrees)
    engEvalString(ep, "rotcam = zeros(2, (nc - 1), 'single');");

    // indicate whether or not to use the fisheye calibration routine (not strictly required).
    engEvalString(ep, "fisheye = false;");

    // indicate whether or not to use the third radial distortion term when doing a projective calibration (not strictly required)
    engEvalString(ep, "k3_enable = false;");

    std::string cam_names;
    cam_names = "cam_names = [";
    std::ostringstream id;
    ROS_DEBUG("number cameras: %d", numberCameras);
    for (int i = 0; i < numberCameras; i++) {
        id << i;
        cam_names = cam_names + "'cam" + id.str() + "_image';";
        id.str("");
        id.clear();
    }
    cam_names = cam_names + "];";

    // the base naming convention for the calibration images (not strictly required), will default to the 'camX_image' convention if not used.
    engEvalString(ep, cam_names.c_str());

    // indicate whether or not to use the batch mode of the stereo calibrator (not strictly required)
    engEvalString(ep, "batch = false;");

    // Perform the calibration
    engEvalString(ep, "auto_multi_calibrator_efficient(camera_vec, input_dir, output_dir, format_image, dX, dY, nx_crnrs, ny_crnrs, proj_tol, rotcam, cam_names, fisheye, k3_enable, batch);");

    // saves the rest of log of matlab
    engEvalString(ep, "diary off;");

    // destroys mxArrays
    mxDestroyArray(slx);
    mxDestroyArray(sly);
    mxDestroyArray(nc);
    mxDestroyArray(nscx);
    mxDestroyArray(nscy);
}



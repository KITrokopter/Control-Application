#include "Position.h"
#include "Calibration.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <sstream>
#include <cstring>
#include "engine.h"
#include "Vector.h"
#include "Line.h"
#include "AmccCalibration.h"

Position::Position()
{
    this->numberCameras = 0;
	Engine *ep;
    // starts a MATLAB process
    if (!(ep = engOpen(""))) {
            fprintf(stderr, "\nCan't start MATLAB engine\n");
    } else {
        this->ep = ep;
    }
    calib = *(new AmccCalibration());
}

Position::Position(Engine *ep)
{
    this->numberCameras = 0;
    this->ep = ep;
    calib = *(new AmccCalibration(ep));
}

Position::setCameras(int number) {
    this->numberCameras = number;
}

Position::calibrate(ChessboardData *chessboardData, int cameraId) {
    calib.multiCameraCalibration(numberCameras, chessboardData->getChessboardWidth(), chessboardData->getChessboardHeight(), chessboardData->getNumberFieldsX(), chessboardData->getNumberFieldsY());
}

Vector Position::updatePosition(Vector v, int cameraId, double quadcopterId) {
    std::string result;
    std::ostringstream id;
    id << camId;
    result = "load('~/multiCalibrationResults/Calib_Results_" + id.str() + ".mat');";
    // loads resulting file in matlab workspace
    engEvalString(ep, result.c_str());
    quad.putVariable("quad", ep);
    engEvalString(ep, "pos = quad * rodrigues(omc_1) + Tc_1;");
    mxArray *position = engGetVariable(ep, "pos");
    Vector pos = *(new Vector(mxGetPr(position)[0], mxGetPr(position)[1], mxGetPr(position)[2]));
    return pos;
}

Vector getPosition(int cameraId) {
    std::string result;
    std::ostringstream id;
    id << cameraId;
    result = "load('~/multiCalibrationResults/Calib_Results_" + id.str() + ".mat');";
    // loads resulting file in matlab workspace
    engEvalString(ep, result.c_str());
    mxArray *tV = engGetVariable(ep, "Tc_1");
    Vector translation = *(new Vector(mxGetPr(tV)[0], mxGetPr(tV)[1], mxGetPr(tV)[2]));
    return translation;
}

Vector getOrientation(int cameraId) {
    std::string result;
    std::ostringstream id;
    id << cameraId;
    result = "load('~/multiCalibrationResults/Calib_Results_" + id.str() + ".mat');";
    // loads resulting file in matlab workspace
    engEvalString(ep, result.c_str());
    mxArray *oV = engGetVariable(ep, "omc_1");
    Vector orientation = *(new Vector(mxGetPr(oV)[0], mxGetPr(oV)[1], mxGetPr(oV)[2]));
    return orientation;
}

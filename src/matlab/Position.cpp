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
#include "Matlab.h"
#include "AmccCalibration.h"
#include <vector>

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

Position::Position(Engine *ep, int numberCameras)
{
    this->numberCameras = numberCameras;
    this->ep = ep;
    calib = *(new AmccCalibration(ep));
}

void Position::calibrate(ChessboardData *chessboardData, int cameraId) {
    calib.multiCameraCalibration(numberCameras, chessboardData->getChessboardWidth(), chessboardData->getChessboardHeight(), chessboardData->getNumberFieldsX(), chessboardData->getNumberFieldsY());
}

void Position::loadValues(int cameraId) {
    std::string result;
    std::ostringstream id;
    id << cameraId;
    result = "load('~/multiCalibrationResults/Calib_Results_" + id.str() + ".mat');";
    // loads resulting file in matlab workspace
    engEvalString(ep, result.c_str());
}

Vector Position::updatePosition(Vector quad, int cameraId, double quadcopterId) {
    loadValues(cameraId);
    quad.putVariable("quad", ep);
    engEvalString(ep, "pos = quad * rodrigues(omc_1) + Tc_1;");
    mxArray *position = engGetVariable(ep, "pos");
    Vector pos = *(new Vector(mxGetPr(position)[0], mxGetPr(position)[1], mxGetPr(position)[2]));
    (quadPos[quadcopterId])[cameraId] = pos;
    if (quadPos[quadcopterId].size() < numberCameras) {
        /// default value
        return *(new Vector(0, 0, 0));
    } else {
        Line *quadPositions = new Line[numberCameras];
        for (int i = 0; i < numberCameras; i++) {
            Vector position = getPosition(cameraId);
            Vector orientation = getOrientation(cameraId);
            quadPositions[i] = *(new Line(position, orientation));
        }
        Matlab *m = new Matlab(ep);
        Vector quadPosition = m->interpolateLines(quadPositions, numberCameras);
        // moving vector = oldPosition - actual position
        Vector movement;
        if (oldPos[quadcopterId].equals(*(new Vector(0, 0, 0)))) {

        } else {
            movement = oldPos[quadcopterId].add(quadPosition.mult(-1));
        }
        oldPos[quadcopterId] = quadPosition;
        quadPos.clear();
        return movement;
    }
}

Vector Position::getPosition(int cameraId) {
    loadValues(cameraId);
    mxArray *tV = engGetVariable(ep, "Tc_1");
    Vector translation = *(new Vector(mxGetPr(tV)[0], mxGetPr(tV)[1], mxGetPr(tV)[2]));
    return translation;
}

Vector Position::getOrientation(int cameraId) {
    loadValues(cameraId);
    mxArray *oV = engGetVariable(ep, "omc_1");
    Vector orientation = *(new Vector(mxGetPr(oV)[0], mxGetPr(oV)[1], mxGetPr(oV)[2]));
    return orientation;
}

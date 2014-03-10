#include "Position.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <sstream>
#include <cstring>
#include "engine.h"
#include "Vector.h"
#include "Line.h"
#include "Matrix.h"
#include "Matlab.h"
#include "TrackingArea.h"
#include "AmccCalibration.h"
#include <vector>
#include <cmath>
#include <math.h>
#include <ros/ros.h>

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
    Vector nan = *(new Vector(NAN, NAN, NAN));
    Matrix nanMatrix = *(new Matrix(NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN));
    // if quadcopter maximal amount is higher than 50, you should change the range of i
    for (int i = 0; i < 50; i++) {
        std::vector<Vector> h(20, nan);
        quadPos.push_back(h);
        oldPos.push_back(nan);
        camCoordCameraPos.push_back(nan);
        camCoordCameraOrient.push_back(nan);
        camRotMat.push_back(nanMatrix);
        realCameraPos.push_back(nan);
        realCameraOrient.push_back(nan);
    }
    rotationMatrix = nanMatrix;
    this->transformed = false;

}

Position::Position(Engine *ep, int numberCameras)
{
    this->numberCameras = numberCameras;
    this->ep = ep;
    Vector nan = *(new Vector(NAN, NAN, NAN));
    Matrix nanMatrix = *(new Matrix(NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN));
    for (int i = 0; i < 50; i++) {
        std::vector<Vector> h(20, nan);
        quadPos.push_back(h);
        oldPos.push_back(nan);
        camCoordCameraPos.push_back(nan);
        camCoordCameraOrient.push_back(nan);
        camRotMat.push_back(nanMatrix);
        realCameraPos.push_back(nan);
        realCameraOrient.push_back(nan);
    }
    rotationMatrix = nanMatrix;
    this->transformed = false;
}

bool Position::calibratedYet(int numberCameras) {
    // trying to load all output files
    mxArray *good;
    std::string load;
    std::ostringstream id;
    load = "try load('/tmp/calibrationResult/Calib_Results_0.mat'); worked = 1; catch worked = 0; end";
    engEvalString(ep, load.c_str());
    good = engGetVariable(ep, "worked");
    double result = mxGetPr(good)[0];
    bool ok = true;
    if (result == 0) {
        ROS_DEBUG("Can't find camera calibration of camera 0");
        ok = false;
    } else {
        for (int i = 1; i < numberCameras; i++) {
            id << i;
            load = "try load('/tmp/calibrationResult/Calib_Results_stereo_0_" + id.str() + ".mat'); worked = 1; catch worked = 0; end";
            engEvalString(ep,load.c_str());
            good = engGetVariable(ep, "worked");
            result = mxGetPr(good)[0];
            if (result == 0) {
                ROS_DEBUG("Can't find camera calibration of camera %d", i);
                ok = false;
            }
            id.str("");
            id.clear();
        }
    }
    return ok;
}

bool Position::calibrate(ChessboardData *chessboardData, int numberCameras) {

    this->numberCameras = numberCameras;
    if (numberCameras < 3) {
        ROS_ERROR("Not enough cameras!");
        return false;
    }

    bool calculated = calibratedYet(numberCameras);

    if (calculated == false) {
        ROS_DEBUG("Calibrating with amcc toolbox\n");
        AmccCalibration *calib = new AmccCalibration(ep);
        calib->multiCameraCalibration(numberCameras, chessboardData->getChessFieldWidth(), chessboardData->getChessFieldHeight(), chessboardData->getNumberCornersX(), chessboardData->getNumberCornersY());
    }

    // checking whether calibration did work (trying to load all output files)
    bool ok = calibratedYet(numberCameras);

    // saves all position and orientation vectors in matlab
    if (ok) {
        for (int i = 0; i < numberCameras; i++) {
            calculateOrientation(i);
            calculatePosition(i);
        }
        Vector v0 = realCameraPos[0];
        Vector d0 = realCameraOrient[0];
        Vector v1 = realCameraPos[1];
        Vector d1 = realCameraOrient[1];
        Vector v2 = realCameraPos[2];
        Vector d2 = realCameraOrient[2];

        ROS_DEBUG("Position camera 0: [%f, %f, %f] is directed in [%f, %f, %f]", v0.getV1(), v0.getV2(), v0.getV3(), d0.getV1(), d0.getV2(), d0.getV3());
        ROS_DEBUG("Position camera 1: [%f, %f, %f] is directed in [%f, %f, %f]", v1.getV1(), v1.getV2(), v1.getV3(), d1.getV1(), d1.getV2(), d1.getV3());
        ROS_DEBUG("Position camera 2: [%f, %f, %f] is directed in [%f, %f, %f]", v2.getV1(), v2.getV2(), v2.getV3(), d2.getV1(), d2.getV2(), d2.getV3());


        ROS_DEBUG("Distance between camera 0 and 1 is %f", v0.add(v1.mult(-1)).getLength());
        ROS_DEBUG("Distance between camera 0 and 2 is %f", v0.add(v2.mult(-1)).getLength());
        ROS_DEBUG("Distance between camera 1 and 2 is %f", v1.add(v2.mult(-1)).getLength());

        //ROS_DEBUG("Calculating tracking area");
        //TrackingArea test = TrackingArea(&realCameraPos, &realCameraOrient, 3, 1500, ep);
        //test.printTrackingArea();
    }

    ROS_INFO("Finished multi camera calibration: %s",(ok)?"true":"false");

    return ok;
}

double Position::getAngle(Vector u, Vector v) {
    // cos(alpha)= u*v/(|u|*|v|)
    double angle = u.scalarMult(v)/(u.getLength() * v.getLength());
    return acos(angle);
}

void Position::angleTry(int sign) {
    // Plain of the cameras E = a + r * u + s * (c - a)
    // as a is always the origin, E intersects the xy-plain in the origin => translation vector is not neccesary
    Matlab *m = new Matlab(ep);

    // a, b, c are the positions of camera 0, 1, 2 in camera coordinate system 0 (might not be calculated yet)
    Vector a = *(new Vector(0, 0, 0));
    loadValues(1);
    mxArray *r = engGetVariable(ep, "T");
    Vector b = *(new Vector(mxGetPr(r)[0], mxGetPr(r)[1], mxGetPr(r)[2]));
    loadValues(2);
    r = engGetVariable(ep, "T");
    Vector c = *(new Vector(mxGetPr(r)[0], mxGetPr(r)[1], mxGetPr(r)[2]));
    mxDestroyArray(r);
    Vector u = b.add(a.mult(-1));
    Vector v = c.add(a.mult(-1));

    Line cameras = *(new Line(a, u));
    // calculates intersection line of plain of cameras in reality and plane of cameras in coordination system

    Vector origin = *(new Vector(1, 1, 0));
    Vector x = *(new Vector(-1, 0, 0));
    // as the method calculates y - origin = (0, -1, 0)
    Vector y = *(new Vector(1, 0, 0));
    Line xAxis = *(new Line(origin, x));
    // works if E isn't already on the x axis or the y axis
    Line intersectionLine = m->getIntersectionLine(cameras, c, xAxis, y);
    //ROS_DEBUG("[%f, %f, %f] + r * [%f, %f, %f]\n", intersectionLine.getA().getV1(), intersectionLine.getA().getV2(), intersectionLine.getA().getV3(), intersectionLine.getU().getV1(), intersectionLine.getU().getV2(), intersectionLine.getU().getV3());


    Vector n = intersectionLine.getU().mult(1/intersectionLine.getU().getLength());
    n.putVariable("n", ep);
    double angle = getAngle(Vector(0, 0, 1), b.cross(c));
    double dataAngle[1] = {sign * angle};
    mxArray *ang = mxCreateDoubleMatrix(1, 1, mxREAL);
    memcpy((void *)mxGetPr(ang), (void *)dataAngle, sizeof(dataAngle));
    engPutVariable(ep, "a", ang);
    engEvalString(ep, "rotationMatrix = [(n(1)^2*(1-cos(a)) + cos(a)), (n(1)*n(2)*(1-cos(a))-n(3) * sin(a)), (n(1)*n(3)*(1-cos(a)) + n(2)*sin(a)); (n(2)*n(1)*(1-cos(a)) + n(3)*sin(a)), (n(2)^2*(1 - cos(a)) + cos(a)), (n(2) * n(3) * (1-cos(a)) - n(1) * sin(a)); (n(3) * n(1) *(1-cos(a)) - n(2) * sin(a)), (n(3) * n(2) * (1 - cos(a)) + n(1) * sin(a)), (n(3)^2 * (1-cos(a)) + cos(a))]");
}

// calculates Vector in the calibration coordinate of camera 0 in the real camera coordination
Vector Position::calculateCoordinateTransformation(Vector w, int cameraId) {
    if (cameraId == -1) {
        Vector nan = *(new Vector(NAN, NAN, NAN));
        return nan;
    } else {
        if (transformed == false) {
            angleTry(1);

            // checking, whether the result of the z value is nearly 0, if you rotate the first camera in coordinate system of camera 0   
            loadValues(1);
            mxArray *r = engGetVariable(ep, "T");
            Vector firstCam = *(new Vector(mxGetPr(r)[0], mxGetPr(r)[1], mxGetPr(r)[2]));
            r = engGetVariable(ep, "rotationMatrix");
            rotationMatrix = *(new Matrix(mxGetPr(r)[0], mxGetPr(r)[1], mxGetPr(r)[2], mxGetPr(r)[3], mxGetPr(r)[4], mxGetPr(r)[5], mxGetPr(r)[6], mxGetPr(r)[7], mxGetPr(r)[8]));
            Vector result = firstCam.aftermult(rotationMatrix);

            ROS_DEBUG("first calculation of transformation matrix, camera 1 would be at position [%f, %f, %f]", result.getV1(), result.getV2(), result.getV3());
            if (!((result.getV3() < 1) && (result.getV3() > -1))) {
                // if value is wrong, the angle has to be negativ
                angleTry(-1);
                r = engGetVariable(ep, "rotationMatrix");
                rotationMatrix = *(new Matrix(mxGetPr(r)[0], mxGetPr(r)[1], mxGetPr(r)[2], mxGetPr(r)[3], mxGetPr(r)[4], mxGetPr(r)[5], mxGetPr(r)[6], mxGetPr(r)[7], mxGetPr(r)[8]));
                result = firstCam.aftermult(rotationMatrix);
                ROS_DEBUG("new calculation has result [%f, %f, %f]", result.getV1(), result.getV2(), result.getV3());
            }
            mxDestroyArray(r);
            this->transformed = true;
        }
        rotationMatrix.printMatrix();
        // calculate rotationMatrix * vector in camera system 0
        return w.aftermult(rotationMatrix);
    }
}

void Position::setNumberCameras(int numberCameras) {
    this->numberCameras = numberCameras;
}

void Position::loadValues(int cameraId) {
    if (cameraId == 0) {
        // loads resulting file in matlab workspace
        engEvalString(ep, "load('/tmp/calibrationResult/Calib_Results_0.mat');");
    } else {
        std::string result;
        std::ostringstream id;
        id << cameraId;
        result = "load('/tmp/calibrationResult/Calib_Results_stereo_0_" + id.str() + ".mat');";
        // loads resulting file in matlab workspace
        engEvalString(ep, result.c_str());
    }
}

Vector Position::updatePosition(Vector quad, int cameraId, int quadcopterId) {
    ROS_DEBUG("update Position: [%f, %f, %f], cameraId: %d", quad.getV1(), quad.getV2(), quad.getV3(), cameraId);
    Vector direction;
    if (cameraId == -1) {
        Vector nan = *(new Vector(NAN, NAN, NAN));
        return nan;
    }

    // rotating coordinate system in coordinate system of camera 0 and then in real coordination system
    // direction = rotationMatrix * (camRotMat * quad)
    direction = (quad.aftermult(camRotMat[cameraId])).aftermult(rotationMatrix);

    // save result
    (quadPos[quadcopterId])[cameraId] = direction;

    // controlling whether all cameras already tracked the quadcopter once
    int valid = 0;
    for (int i = 0; i < numberCameras; i++) {
        if (quadPos[quadcopterId][i].isValid()) {
            valid++;
        }
    }
    if (valid != numberCameras) {
        // default value, when not all cameras tracked it yet
        ROS_DEBUG("Not all cameras did track quadcopter %d yet. Camera %d tracked it at position [%f, %f, %f]\n", quadcopterId, cameraId, direction.getV1(), direction.getV2(), direction.getV3());
        Vector nan = *(new Vector(NAN, NAN, NAN));
        return nan;
    } else {
        // not calculated before, first time calculating
        if (!(oldPos[quadcopterId].isValid())) {
            ROS_DEBUG("First calculation of position");

            // building lines from camera position to quadcopter position
            Line *quadPositions = new Line[numberCameras];
            for (int i = 0; i < numberCameras; i++) {
                Vector camPos = getPosition(i);
                quadPositions[i] = *(new Line(camPos, quadPos[quadcopterId][i]));
                printf("%d: [%f, %f, %f] + r * [%f, %f, %f]\n", i, camPos.getV1(), camPos.getV2(), camPos.getV3(), quadPos[quadcopterId][i].getV1(), quadPos[quadcopterId][i].getV2(), quadPos[quadcopterId][i].getV3());
            }

            Matlab *m = new Matlab(ep);

            Vector quadPosition = m->interpolateLines(quadPositions, numberCameras);

            oldPos[quadcopterId] = quadPosition;
            ROS_DEBUG("First seen position of quadcopter %d is [%f, %f, %f]\n", quadcopterId, quadPosition.getV1(), quadPosition.getV2(), quadPosition.getV3());

            return quadPosition;
        } else {
            ROS_DEBUG("New calculation of position");
            Matlab *m = new Matlab(ep);

            // interpolation factor should be tested.
            Vector position = getPosition(cameraId);
            // line from camera to tracked object
            Line tracked = Line(position, direction);
            // calulating actual pos
            Vector newPos = m->interpolateLine(tracked, oldPos[quadcopterId], 0.5);
            // saving new Pos
            ROS_DEBUG("New position of quadcopter %d is [%f, %f, %f]\n", quadcopterId, newPos.getV1(), newPos.getV2(), newPos.getV3());
            oldPos[quadcopterId] = newPos;
            return newPos;
        }
    }
}

void Position::calculatePosition(int cameraId) {
    if (cameraId != -1) {
        if (cameraId != 0) {
            loadValues(cameraId);
            mxArray *r = engGetVariable(ep, "T");
            camCoordCameraPos[cameraId] = *(new Vector(mxGetPr(r)[0], mxGetPr(r)[1], mxGetPr(r)[2]));
            mxDestroyArray(r);
        } else {
            // camera 0 is at the origin and looks down the positive z axis
            camCoordCameraPos[0] = *(new Vector(0, 0, 0));
        }
        realCameraPos[cameraId] = calculateCoordinateTransformation(camCoordCameraPos[cameraId], cameraId);
    }
}

Vector Position::getPosition(int cameraId) {
    return this->realCameraPos[cameraId];
}

void Position::calculateOrientation(int cameraId) {
    if (cameraId != -1) {
        if (cameraId != 0) {
            loadValues(cameraId);
            mxArray *r = engGetVariable(ep, "R");
            camRotMat[cameraId] = *(new Matrix(mxGetPr(r)[0], mxGetPr(r)[1], mxGetPr(r)[2], mxGetPr(r)[3], mxGetPr(r)[4], mxGetPr(r)[5], mxGetPr(r)[6], mxGetPr(r)[7], mxGetPr(r)[8]));
            engEvalString(ep, "R = rodrigues(R)");
            camCoordCameraOrient[cameraId] = *(new Vector(mxGetPr(r)[0], mxGetPr(r)[1], mxGetPr(r)[2]));
        } else {
            // camera 0 is at the origin and looks down the positive z axis
            camRotMat[0] = *(new Matrix(1, 0, 0, 0, 1, 0, 0, 0, 1));
            camCoordCameraOrient[0] = *(new Vector(0, 0, 1));
        }
        realCameraOrient[cameraId] = camCoordCameraOrient[cameraId].aftermult(rotationMatrix);
    }
}

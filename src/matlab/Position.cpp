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
#include "profiling.hpp"

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
    initialize();
}

Position::Position(Engine *ep, int numberCameras)
{
    this->numberCameras = numberCameras;
    this->ep = ep;
    initialize();
}

void Position::initialize() {
    this->transformed = false;
    this->interpolationDependent = true;
    distance = 0;
    Vector nan = Vector(NAN, NAN, NAN);
    Matrix nanMatrix = Matrix(NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN);

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
        imageAge.push_back(0);
    }

    rotationMatrix = nanMatrix;
    if (interpolationDependent) {
        ROS_INFO("Using adaptive interpolation factor.");
    } else {
        ROS_INFO("Using always the same interpolation factor.");
    }
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
        // calculates from the back as for camera 0 the rotationmatrix isn't calculated yet
        for (int i = (numberCameras - 1); i >= 0; i--) {
            calculatePosition(i);
            calculateOrientation(i);
        }

        Vector v0 = realCameraPos[0];
        Vector d0 = realCameraOrient[0];
        Vector v1 = realCameraPos[1];
        Vector d1 = realCameraOrient[1];
        Vector v2 = realCameraPos[2];
        Vector d2 = realCameraOrient[2];

        ROS_DEBUG("Position camera 0: [%.2f, %.2f, %.2f] is directed in [%.2f, %.2f, %.2f]", v0.getV1(), v0.getV2(), v0.getV3(), d0.getV1(), d0.getV2(), d0.getV3());
        ROS_DEBUG("Position camera 1: [%.2f, %.2f, %.2f] is directed in [%.2f, %.2f, %.2f]", v1.getV1(), v1.getV2(), v1.getV3(), d1.getV1(), d1.getV2(), d1.getV3());
        ROS_DEBUG("Position camera 2: [%.2f, %.2f, %.2f] is directed in [%.2f, %.2f, %.2f]", v2.getV1(), v2.getV2(), v2.getV3(), d2.getV1(), d2.getV2(), d2.getV3());

        ROS_DEBUG("Distance between camera 0 and 1 is %.2f", v0.add(v1.mult(-1)).getLength());
        ROS_DEBUG("Distance between camera 0 and 2 is %.2f", v0.add(v2.mult(-1)).getLength());
        ROS_DEBUG("Distance between camera 1 and 2 is %.2f", v1.add(v2.mult(-1)).getLength());

        ROS_DEBUG("Calculating tracking area");
        setTrackingArea(2000);
        tracking.printTrackingArea();
    }

    ROS_INFO("Finished multi camera calibration: %s",(ok)?"true":"false");

    return ok;
}

double Position::getAngle(Vector u, Vector v) {
    // cos(alpha)= u*v/(|u|*|v|)
    double angle = u.scalarMult(v)/(u.getLength() * v.getLength());
    angle = acos(angle);

    // checks whether angle is between 0 and 90 degree
    if (angle > M_PI/2) {
        angle = -(angle - M_PI);
    }
    return angle;
}

void Position::angleTry(int sign) {
    // Plain of the cameras E = a + r * u + s * (c - a)
    // as a is always the origin, E intersects the xy-plain in the origin => translation vector is not neccesary
    Matlab *m = new Matlab(ep);

    // a, b, c are the positions of camera 0, 1, 2 in camera coordinate system 0 (might not be calculated yet)
    Vector a = Vector(0, 0, 0);
    loadValues(1);
    mxArray *r = engGetVariable(ep, "T");
    Vector b = Vector(mxGetPr(r)[0], mxGetPr(r)[1], mxGetPr(r)[2]);
    loadValues(2);
    r = engGetVariable(ep, "T");
    Vector c = Vector(mxGetPr(r)[0], mxGetPr(r)[1], mxGetPr(r)[2]);
    mxDestroyArray(r);
    Vector u = b.add(a.mult(-1));
    Vector v = c.add(a.mult(-1));

    Line cameras = Line(a, u);
    // calculates intersection line of plain of cameras in reality and plane of cameras in coordination system

    Vector origin = Vector(1, 1, 0);
    Vector x = Vector(-1, 0, 0);
    // as the method calculates y - origin = (0, -1, 0)
    Vector y = Vector(1, 0, 0);
    Line xAxis = Line(origin, x);
    // works if E isn't already on the x axis or the y axis
    Line intersectionLine = m->getIntersectionLine(cameras, c, xAxis, y);
    //ROS_DEBUG("[%f, %f, %f] + r * [%f, %f, %f]\n", intersectionLine.getA().getV1(), intersectionLine.getA().getV2(), intersectionLine.getA().getV3(), intersectionLine.getU().getV1(), intersectionLine.getU().getV2(), intersectionLine.getU().getV3());

    Vector n = intersectionLine.getU().mult(1/intersectionLine.getU().getLength());
    if (n.getV3() < 0) {
        n.mult(-1);
    }

    n.putVariable("n", ep);
    double angle = getAngle(Vector(0, 0, 1), b.cross(c));
    double dataAngle[1] = {sign * angle};
    mxArray *ang = mxCreateDoubleMatrix(1, 1, mxREAL);
    memcpy((void *)mxGetPr(ang), (void *)dataAngle, sizeof(dataAngle));
    engPutVariable(ep, "a", ang);
    engEvalString(ep, "rotationMatrix = [(n(1)^2*(1-cos(a)) + cos(a)), (n(1)*n(2)*(1-cos(a))-n(3) * sin(a)), (n(1)*n(3)*(1-cos(a)) + n(2)*sin(a)); (n(2)*n(1)*(1-cos(a)) + n(3)*sin(a)), (n(2)^2*(1 - cos(a)) + cos(a)), (n(2) * n(3) * (1-cos(a)) - n(1) * sin(a)); (n(3) * n(1) *(1-cos(a)) - n(2) * sin(a)), (n(3) * n(2) * (1 - cos(a)) + n(1) * sin(a)), (n(3)^2 * (1-cos(a)) + cos(a))]");
}

// calculates Vector in the calibration coordinate of camera 0 in the real camera coordination
Vector Position::calculateCoordinateTransformation(Vector w) {
    if (transformed == false) {
        angleTry(1);

        // checking, whether the result of the z value is nearly 0, if you rotate the first camera in coordinate system of camera 0
        loadValues(1);
        mxArray *r = engGetVariable(ep, "T");
        Vector firstCam = Vector(mxGetPr(r)[0], mxGetPr(r)[1], mxGetPr(r)[2]);
        r = engGetVariable(ep, "rotationMatrix");
        rotationMatrix = Matrix(mxGetPr(r)[0], mxGetPr(r)[3], mxGetPr(r)[6], mxGetPr(r)[1], mxGetPr(r)[4], mxGetPr(r)[7], mxGetPr(r)[2], mxGetPr(r)[5], mxGetPr(r)[8]);
        Vector result = firstCam.aftermult(rotationMatrix);

        if (!((result.getV3() < 0.5) && (result.getV3() > -0.5))) {
            ROS_DEBUG("first calculation of transformation matrix, camera 1 would be at position [%.2f, %.2f, %.2f]", result.getV1(), result.getV2(), result.getV3());
            // if value is wrong, the angle has to be negativ
            angleTry(-1);
            r = engGetVariable(ep, "rotationMatrix");
            rotationMatrix = Matrix(mxGetPr(r)[0], mxGetPr(r)[3], mxGetPr(r)[6], mxGetPr(r)[1], mxGetPr(r)[4], mxGetPr(r)[7], mxGetPr(r)[2], mxGetPr(r)[5], mxGetPr(r)[8]);
            result = firstCam.aftermult(rotationMatrix);
            ROS_DEBUG("new calculation has result [%.2f, %.2f, %.2f]", result.getV1(), result.getV2(), result.getV3());

            if (!((result.getV3() < 0.5) && (result.getV3() > -0.5))) {
                ROS_ERROR("Transformation didn't work!");
                return Vector(NAN, NAN, NAN);
            }
        }
        mxDestroyArray(r);
        this->transformed = true;
    }
    // calculate rotationMatrix * vector in camera system 0
    return w.aftermult(rotationMatrix);
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

Vector Position::updatePosition(std::vector<CameraData> cameraLines) {

    int quadcopterId = cameraLines[0].quadcopterId;

    for(int i = 1; i < cameraLines.size(); i++) {
        if (cameraLines[i].quadcopterId != quadcopterId) {
            ROS_ERROR("scheduler passes datas of different quadcopter ids.");
        }
    }

    // increments counter for other cameras
    for (int i = 0; i < numberCameras; i++) {
        bool tracked = false;
        for(int j = 0; j < cameraLines.size(); j++) {
            if (i == cameraLines[j].camNo) {
                tracked = true;
            }
        }
        if (!(tracked)) {
            imageAge[i]++;
        }
    }
    // resets counter of cameras that are tracking quadcopter
    for(int i = 0; i < cameraLines.size(); i++) {
        imageAge[cameraLines[i].camNo] = 0;
    }

    Vector* direction = new Vector[cameraLines.size()];

    // rotating coordinate system in coordinate system of camera 0 and then in real coordination system
    // direction = rotationMatrix * (camRotMat * quad)
    for (int i = 0; i < cameraLines.size(); i++) {
        direction[i] = (cameraLines[i].cameraVector.aftermult(camRotMat[cameraLines[i].camNo])).aftermult(rotationMatrix);
        // save result
        (quadPos[quadcopterId])[cameraLines[i].camNo] = direction[i];
    }


    // controlling whether all cameras already tracked the quadcopter once
    int valid = 0;
    for (int i = 0; i < numberCameras; i++) {
        if (quadPos[quadcopterId][i].isValid()) {
            valid++;
        }
    }

    if (valid != numberCameras) {
        // default value, when not all cameras tracked it yet
        ROS_DEBUG("Not all cameras did track quadcopter %d yet.\n", quadcopterId);
        Vector nan = Vector(NAN, NAN, NAN);
        return nan;
    } else {
        if (!(oldPos[quadcopterId].isValid())) {
            // not calculated before, first time calculating
            int tooOld = 0;
            for (int i = 0; i < numberCameras; i++) {
                if (imageAge[i] > 5) {
                    ROS_DEBUG("Information of camera %d is too old.", i);
                    tooOld++;
                }
            }
            if (2 > numberCameras - tooOld) {
                ROS_DEBUG("Information can't be used, as too much cameras can't track it anymore");
                return Vector(NAN, NAN, NAN);
            }

            ROS_DEBUG("First calculation of position with camera information:");

            // building lines from camera position to quadcopter position
            Line *quadPositions = new Line[numberCameras];
            for (int i = 0; i < numberCameras; i++) {
                Vector camPos = getPosition(i);
                quadPositions[i] = Line(camPos, quadPos[quadcopterId][i]);
            }

            Matlab *m = new Matlab(ep);
            long int startTime = getNanoTime();
            Vector quadPosition = m->interpolateLines(quadPositions, numberCameras);
            long int endTime = getNanoTime();
            ROS_DEBUG("Calculation was %.3f long", (endTime - startTime) / 1e9);

            oldPos[quadcopterId] = quadPosition;
            ROS_DEBUG("First seen position of quadcopter %d is [%f, %f, %f]", quadcopterId, quadPosition.getV1(), quadPosition.getV2(), quadPosition.getV3());
            /*if (tracked(quadPosition)) {
                ROS_DEBUG("In tracking area");
            } else {
                ROS_DEBUG("Not in tracking area");
            }*/

            // as distance of 150 has interpolation factor 0.5
            distance = 150;
            return quadPosition;
        } else {
            // calculated before

            int tooOld = 0;
            for (int i = 0; i < numberCameras; i++) {
                if (imageAge[i] > 10) {
                    ROS_DEBUG("Information of camera %d is too old.", i);
                    tooOld++;
                }
            }

            if (2 > numberCameras - tooOld) {
                ROS_WARN("Only camera %d still tracks quadcopter %d.", cameraLines[0].camNo, cameraLines[0].quadcopterId);
            }

            Matlab *m = new Matlab(ep);
            Vector newPos;

            if (cameraLines.size() > 1) {
                Line* tracking = new Line[cameraLines.size()];
                for (int i = 0; i < cameraLines.size(); i++) {
                    tracking[i] = Line(getPosition(cameraLines[i].camNo), cameraLines[i].cameraVector);
                }
                newPos = m->interpolateLines(tracking, cameraLines.size());
            } else {
                Vector position = getPosition(cameraLines[0].camNo);
                Line tracked = Line(position, direction[0]);

                // calulating actual pos
                if (interpolationDependent) {
                    if (distance > 200) {
                        newPos = m->interpolateLine(tracked, oldPos[quadcopterId], 0.7);
                    } else if (distance < 100) {
                        newPos = m->interpolateLine(tracked, oldPos[quadcopterId], 0.2);
                    } else {
                        // diff is between 0 and 100
                        double diff = distance - 100;
                        // diff is between 0.2 and 0.7
                        diff = 0.2 + 0.5 * diff/100.0;
                        newPos = m->interpolateLine(tracked, oldPos[quadcopterId], diff);
                    }
                } else {
                    newPos = m->interpolateLine(tracked, oldPos[quadcopterId], 0.5);
                }
            }

            // calculates distance between last seen position and new calculated position
            distance = (oldPos[quadcopterId]).add(newPos.mult(-1)).getLength();

            // saving new Pos
            ROS_DEBUG("New position of quadcopter %d is [%f, %f, %f]", quadcopterId, newPos.getV1(), newPos.getV2(), newPos.getV3());
            oldPos[quadcopterId] = newPos;
            /*if (tracked(newPos)) {
                ROS_DEBUG("In tracking area");
            } else {
                ROS_DEBUG("Not in tracking area");
            }*/
            return newPos;
        }
    }
}

bool Position::tracked(Vector quadPos) {
    if (tracking.inCameraRange(realCameraPos, realCameraOrient, numberCameras, 2000, quadPos, ep)) {
        return true;
    } else {
        return false;
    }
}

void Position::calculatePosition(int cameraId) {
    if (cameraId != -1) {
        if (cameraId != 0) {
            loadValues(cameraId);
            mxArray *r = engGetVariable(ep, "T");
            camCoordCameraPos[cameraId] = Vector(mxGetPr(r)[0], mxGetPr(r)[1], mxGetPr(r)[2]);
            mxDestroyArray(r);
        } else {
            // camera 0 is at the origin and looks down the positive z axis
            camCoordCameraPos[0] = Vector(0, 0, 0);
        }
        realCameraPos[cameraId] = calculateCoordinateTransformation(camCoordCameraPos[cameraId]);
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
            camRotMat[cameraId] = Matrix(mxGetPr(r)[0], mxGetPr(r)[3], mxGetPr(r)[6], mxGetPr(r)[1], mxGetPr(r)[4], mxGetPr(r)[7], mxGetPr(r)[2], mxGetPr(r)[5], mxGetPr(r)[8]);
            // camRotMat * [0, 0, 1]
            camCoordCameraOrient[cameraId] = (Vector(0, 0, 1)).aftermult(camRotMat[cameraId]);
        } else {
            // camera 0 is at the origin and looks down the positive z axis
            camRotMat[0] = Matrix(1, 0, 0, 0, 1, 0, 0, 0, 1);
            camCoordCameraOrient[0] = Vector(0, 0, 1);
        }
        realCameraOrient[cameraId] = camCoordCameraOrient[cameraId].aftermult(rotationMatrix);
    }
}

void Position::setTrackingArea(double maxRange) {
    this->tracking = TrackingArea(realCameraPos, realCameraOrient, numberCameras, maxRange, ep);
}

TrackingArea Position::getTrackingArea() {
    return this->tracking;
}

double Position::getDistance() {
    return this->distance;
}

cv::Mat Position::getDistortionCoefficients(int cameraId) {
    std::string result;
    std::ostringstream id;
    id << cameraId;
    result = "load('/tmp/calibrationResult/Calib_Results_" + id.str() + ".mat');";
    // loads resulting file in matlab workspace
    engEvalString(ep, result.c_str());
	
    mxArray *r = engGetVariable(ep, "kc");
	
	cv::Mat distortionCoefficients(cv::Size(5, 1), CV_64F);
    distortionCoefficients.at<double>(0) = mxGetPr(r)[0];
	distortionCoefficients.at<double>(1) = mxGetPr(r)[1];
	distortionCoefficients.at<double>(2) = mxGetPr(r)[2];
	distortionCoefficients.at<double>(3) = mxGetPr(r)[3];
	distortionCoefficients.at<double>(4) = mxGetPr(r)[4];
	
    mxDestroyArray(r);
	
	return distortionCoefficients;
}

cv::Mat Position::getIntrinsicsMatrix(int cameraId) {
    std::string result;
    std::ostringstream id;
    id << cameraId;
    result = "load('/tmp/calibrationResult/Calib_Results_" + id.str() + ".mat');";
    // loads resulting file in matlab workspace
    engEvalString(ep, result.c_str());
    mxArray *fc = engGetVariable(ep, "fc");
    mxArray *alpha = engGetVariable(ep, "alpha_c");
    mxArray *cc = engGetVariable(ep, "cc");
    double fc1 = mxGetPr(fc)[0];
	
    cv::Mat intrinsicsMatrix(cv::Size(3, 3), CV_64F);
	intrinsicsMatrix.at<double>(0, 0) = fc1;
	intrinsicsMatrix.at<double>(0, 1) = mxGetPr(alpha)[0] * fc1;
	intrinsicsMatrix.at<double>(0, 2) = mxGetPr(cc)[0];
	intrinsicsMatrix.at<double>(1, 0) = 0;
	intrinsicsMatrix.at<double>(1, 1) = mxGetPr(fc)[1];
	intrinsicsMatrix.at<double>(1, 2) = mxGetPr(cc)[1];
	intrinsicsMatrix.at<double>(2, 0) = 0;
	intrinsicsMatrix.at<double>(2, 1) = 0;
	intrinsicsMatrix.at<double>(2, 2) = 1;

    mxDestroyArray(fc);
    mxDestroyArray(cc);
    mxDestroyArray(alpha);
	
    return intrinsicsMatrix;
}

Matrix Position::getRotationMatrix(int cameraId) {
    Matrix rotation = rotationMatrix.multiplicate(camRotMat[cameraId]);
    return rotation;
}

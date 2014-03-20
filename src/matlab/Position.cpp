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
    distance = 0;
    this->interpolationDependent = false;
    initialize();
}

Position::Position(bool interpolationDependent)
{
    this->numberCameras = 0;
    Engine *ep;
    // starts a MATLAB process
    if (!(ep = engOpen(""))) {
            fprintf(stderr, "\nCan't start MATLAB engine\n");
    } else {
        this->ep = ep;
    }
    this->interpolationDependent = interpolationDependent;
    initialize();
}

Position::Position(Engine *ep, int numberCameras)
{
    this->numberCameras = numberCameras;
    this->ep = ep;
    this->interpolationDependent = false;
    initialize();
}

Position::Position(Engine *ep, int numberCameras, bool interpolationDependent)
{
    this->numberCameras = numberCameras;
    this->ep = ep;
    this->interpolationDependent = interpolationDependent;
    initialize();
}

void Position::initialize() {
    this->transformed = false;
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

        ROS_DEBUG("Position camera 0: [%f, %f, %f] is directed in [%f, %f, %f]", v0.getV1(), v0.getV2(), v0.getV3(), d0.getV1(), d0.getV2(), d0.getV3());
        ROS_DEBUG("Position camera 1: [%f, %f, %f] is directed in [%f, %f, %f]", v1.getV1(), v1.getV2(), v1.getV3(), d1.getV1(), d1.getV2(), d1.getV3());
        ROS_DEBUG("Position camera 2: [%f, %f, %f] is directed in [%f, %f, %f]", v2.getV1(), v2.getV2(), v2.getV3(), d2.getV1(), d2.getV2(), d2.getV3());
        //printf("Position camera 0: [%f, %f, %f] is directed in [%f, %f, %f]\n", v0.getV1(), v0.getV2(), v0.getV3(), d0.getV1(), d0.getV2(), d0.getV3());
        //printf("Position camera 1: [%f, %f, %f] is directed in [%f, %f, %f]\n", v1.getV1(), v1.getV2(), v1.getV3(), d1.getV1(), d1.getV2(), d1.getV3());
        //printf("Position camera 2: [%f, %f, %f] is directed in [%f, %f, %f]\n", v2.getV1(), v2.getV2(), v2.getV3(), d2.getV1(), d2.getV2(), d2.getV3());


        ROS_DEBUG("Distance between camera 0 and 1 is %f", v0.add(v1.mult(-1)).getLength());
        ROS_DEBUG("Distance between camera 0 and 2 is %f", v0.add(v2.mult(-1)).getLength());
        ROS_DEBUG("Distance between camera 1 and 2 is %f", v1.add(v2.mult(-1)).getLength());

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
Vector Position::calculateCoordinateTransformation(Vector w, int cameraId) {
    if (cameraId == -1) {
        Vector nan = Vector(NAN, NAN, NAN);
        return nan;
    } else {
        if (transformed == false) {
            angleTry(1);

            // checking, whether the result of the z value is nearly 0, if you rotate the first camera in coordinate system of camera 0   
            loadValues(1);
            mxArray *r = engGetVariable(ep, "T");
            Vector firstCam = Vector(mxGetPr(r)[0], mxGetPr(r)[1], mxGetPr(r)[2]);
            r = engGetVariable(ep, "rotationMatrix");
            rotationMatrix = Matrix(mxGetPr(r)[0], mxGetPr(r)[3], mxGetPr(r)[6], mxGetPr(r)[1], mxGetPr(r)[4], mxGetPr(r)[7], mxGetPr(r)[2], mxGetPr(r)[5], mxGetPr(r)[8]);
            Vector result = firstCam.aftermult(rotationMatrix);

            ROS_DEBUG("first calculation of transformation matrix, camera 1 would be at position [%f, %f, %f]", result.getV1(), result.getV2(), result.getV3());
            if (!((result.getV3() < 0.5) && (result.getV3() > -0.5))) {
                // if value is wrong, the angle has to be negativ
                angleTry(-1);
                r = engGetVariable(ep, "rotationMatrix");
                rotationMatrix = Matrix(mxGetPr(r)[0], mxGetPr(r)[3], mxGetPr(r)[6], mxGetPr(r)[1], mxGetPr(r)[4], mxGetPr(r)[7], mxGetPr(r)[2], mxGetPr(r)[5], mxGetPr(r)[8]);
                result = firstCam.aftermult(rotationMatrix);
                ROS_DEBUG("new calculation has result [%f, %f, %f]", result.getV1(), result.getV2(), result.getV3());
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
    // increments counter for other cameras
    for (int i = 0; i < numberCameras; i++) {
        if (i != cameraId) {
            imageAge[i]++;
        }
    }
    // resets counter of camera with cameraId
    imageAge[cameraId] = 0;

    Vector direction;
    if (cameraId == -1) {
        return Vector(NAN, NAN, NAN);
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
                printf("%d: [%f, %f, %f] + r * [%f, %f, %f]\n", i, camPos.getV1(), camPos.getV2(), camPos.getV3(), quadPos[quadcopterId][i].getV1(), quadPos[quadcopterId][i].getV2(), quadPos[quadcopterId][i].getV3());
            }

            Matlab *m = new Matlab(ep);

            Vector quadPosition = m->interpolateLines(quadPositions, numberCameras);

            oldPos[quadcopterId] = quadPosition;
            ROS_DEBUG("First seen position of quadcopter %d is [%f, %f, %f]", quadcopterId, quadPosition.getV1(), quadPosition.getV2(), quadPosition.getV3());
            return quadPosition;
        } else {
            // not calculated before, first time calculating
            int tooOld = 0;
            for (int i = 0; i < numberCameras; i++) {
                if (imageAge[i] > 20) {
                    ROS_DEBUG("Information of camera %d is too old.", i);
                    tooOld++;
                }
            }

            if (2 > numberCameras - tooOld) {
                ROS_DEBUG("Information can't be used, as too much cameras can't track it anymore");
                return Vector(NAN, NAN, NAN);
            }

            Matlab *m = new Matlab(ep);

            // interpolation factor should be tested.
            Vector position = getPosition(cameraId);
            // line from camera to tracked object
            Line tracked = Line(position, direction);
            Vector newPos;

            // calulating actual pos
            if ((interpolationDependent) && (distance != 0)) {
                if (distance > 210) {
                    newPos = m->interpolateLine(tracked, oldPos[quadcopterId], 0.7);
                } else if (distance < 10) {
                    newPos = m->interpolateLine(tracked, oldPos[quadcopterId], 0.2);
                } else {
                    // diff is between 0 and 100
                    double diff = (distance - 10) / 2;
                    // diff is between 0.2 and 0.7
                    diff = 0.2 + 0.5 * diff/100.0;
                    newPos = m->interpolateLine(tracked, oldPos[quadcopterId], diff);
                }
            } else {
                newPos = m->interpolateLine(tracked, oldPos[quadcopterId], 0.5);
            }

            // calculates distance between last seen position and new calculated position
            distance = (oldPos[quadcopterId]).add(newPos.mult(-1)).getLength();

            // saving new Pos
            ROS_DEBUG("New position of quadcopter %d seen of camera %d is [%f, %f, %f]", quadcopterId, cameraId, newPos.getV1(), newPos.getV2(), newPos.getV3());
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
            camCoordCameraPos[cameraId] = Vector(mxGetPr(r)[0], mxGetPr(r)[1], mxGetPr(r)[2]);
            mxDestroyArray(r);
        } else {
            // camera 0 is at the origin and looks down the positive z axis
            camCoordCameraPos[0] = Vector(0, 0, 0);
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
            camRotMat[cameraId] = Matrix(mxGetPr(r)[0], mxGetPr(r)[3], mxGetPr(r)[6], mxGetPr(r)[1], mxGetPr(r)[4], mxGetPr(r)[7], mxGetPr(r)[2], mxGetPr(r)[5], mxGetPr(r)[8]);
            //engEvalString(ep, "R = rodrigues(R)");
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
	
// 	for (int i = 0; i < 5; i++) {
// 		ROS_DEBUG("DistortionCoefficients %d: %f", i, distortionCoefficients.at<double>(i));
// 	}
	
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
	
// 	for (int i = 0; i < 9; i++) {
// 		ROS_DEBUG("IntrinsicsMatrix %d: %f", i, intrinsicsMatrix.at<double>(i));
// 	}
	
    mxDestroyArray(fc);
    mxDestroyArray(cc);
    mxDestroyArray(alpha);
	
    return intrinsicsMatrix;
}

Matrix Position::getRotationMatrix(int cameraId) {
    Matrix rotation = rotationMatrix.multiplicate(camRotMat[cameraId]);
    return rotation;
}

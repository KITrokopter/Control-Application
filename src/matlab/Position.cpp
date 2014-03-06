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
#include "Matlab.h"
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
    // if quadcopter maximal amount is higher than 50, you should change the range of i
    for (int i = 0; i < 50; i++) {
        std::vector<Vector> h(20, nan);
        quadPos.push_back(h);
        oldPos.push_back(nan);
    }
    this->transformed = false;
    this->calibrated = false;

}

Position::Position(Engine *ep, int numberCameras)
{
    this->numberCameras = numberCameras;
    this->ep = ep;
    Vector nan = *(new Vector(NAN, NAN, NAN));
    for (int i = 0; i < 50; i++) {
        std::vector<Vector> h(20, nan);
        quadPos.push_back(h);
        oldPos.push_back(nan);
    }
    this->transformed = false;
    this->calibrated = false;
}

bool Position::calibrate(ChessboardData *chessboardData, int numberCameras) {

    this->numberCameras = numberCameras;
    if (numberCameras < 3) {
        ROS_ERROR("Not enough cameras!");
        return false;
    }
    
    AmccCalibration *calib = new AmccCalibration(ep);
    calib->multiCameraCalibration(numberCameras, chessboardData->getChessFieldWidth(), chessboardData->getChessFieldHeight(), chessboardData->getNumberCornersX(), chessboardData->getNumberCornersY());

    // checking whether calibration did work (trying to load all output files)
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
            engEvalString(ep, load.c_str());
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

    // saves all position and orientation vectors in matlab
    if (ok) {
        for (int i = 0; i < numberCameras; i++) {
            getOrientation(i);
            getPosition(i);
        }
        this->calibrated = true;
        Vector v0 = getPosition(0);
        Vector v1 = getPosition(1);
        Vector v2 = getPosition(2);

        ROS_DEBUG("Position camera 0: [%f, %f, %f]", v0.getV1(), v0.getV2(), v0.getV3());
        ROS_DEBUG("Position camera 1: [%f, %f, %f]", v1.getV1(), v1.getV2(), v1.getV3());
        ROS_DEBUG("Position camera 2: [%f, %f, %f]", v2.getV1(), v2.getV2(), v2.getV3());


        ROS_DEBUG("Distance between camera 0 and 1 is %f", v0.add(v1.mult(-1)).getLength());
        ROS_DEBUG("Distance between camera 0 and 2 is %f", v0.add(v2.mult(-1)).getLength());
        ROS_DEBUG("Distance between camera 1 and 2 is %f", v1.add(v2.mult(-1)).getLength());
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
    Vector a = getPositionInCameraCoordination(0);
    Vector b = getPositionInCameraCoordination(1);
    Vector c = getPositionInCameraCoordination(2);
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
    //printf("[%f, %f, %f] + r * [%f, %f, %f]\n", intersectionLine.getA().getV1(), intersectionLine.getA().getV2(), intersectionLine.getA().getV3(), intersectionLine.getU().getV1(), intersectionLine.getU().getV2(), intersectionLine.getU().getV3());


    Vector n = intersectionLine.getU().mult(1/intersectionLine.getU().getLength());
    n.putVariable("n", ep);
    double angle = getAngle(*(new Vector(0, 0, 1)), b.cross(c));
    double dataAngle[1] = {sign * angle};
    mxArray *ang = mxCreateDoubleMatrix(1, 1, mxREAL);
    memcpy((void *)mxGetPr(ang), (void *)dataAngle, sizeof(dataAngle));
    engPutVariable(ep, "a", ang);
    engEvalString(ep, "rotationMatrix = [(n(1)^2*(1-cos(a)) + cos(a)), (n(1)*n(2)*(1-cos(a))-n(3) * sin(a)), (n(1)*n(3)*(1-cos(a)) + n(2)*sin(a)); (n(2)*n(1)*(1-cos(a)) + n(3)*sin(a)), (n(2)^2*(1 - cos(a)) + cos(a)), (n(2) * n(3) * (1-cos(a)) - n(1) * sin(a)); (n(3) * n(1) *(1-cos(a)) - n(2) * sin(a)), (n(3) * n(2) * (1 - cos(a)) + n(1) * sin(a)), (n(3)^2 * (1-cos(a)) + cos(a))]");
}

// calculates Vector in the calibration coordinate of camera 0 in the real camera coordination
Vector Position::getCoordinationTransformation(Vector w, int cameraId) {
    if (cameraId == -1) {
        Vector nan = *(new Vector(NAN, NAN, NAN));
        return nan;
    } else {
        if (transformed == false) {
            angleTry(1);

            // checking, whether the result of the z value is nearly 0, if you rotate the first camera in coordinate system of camera 0
            Vector firstCam = getPositionInCameraCoordination(1);
            firstCam.putVariable("firstCam", ep);
            engEvalString(ep, "result = rotationMatrix * firstCam';");
            mxArray* result = engGetVariable(ep, "result");
            if (!((mxGetPr(result)[2] < 0.5) && (mxGetPr(result)[2] > -0.5))) {
                // if value is wrong, the angle has to be negativ
                angleTry(-1);
            }
            this->transformed = true;
       }

        w.putVariable("cameraCoordinationPos", ep);
        // calculate rotationMatrix * vector in camera system 0
        engEvalString(ep, "result = rotationMatrix * cameraCoordinationPos';");
        mxArray* result = engGetVariable(ep, "result");
        Vector r = *(new Vector(mxGetPr(result)[0], mxGetPr(result)[1], mxGetPr(result)[2]));
        mxDestroyArray(result);
        return r;
    }

}

void Position::setNumberCameras(int numberCameras) {
    this->numberCameras = numberCameras;
}

void Position::loadValues(int cameraId) {
    int i;
    mxArray* notSupp;
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
    Vector direction;
    if (cameraId == -1) {
        Vector nan = *(new Vector(NAN, NAN, NAN));
        return nan;
    }

    /*
     * orientation = rotationMatrix * quad
     * Line: getPosition(cameraId) + r (rotationMatrix * quad)
     */

    // rotating coordinate system in coordinate system of camera 0 and then in real coordination system
    else if (cameraId != 0) {
        std::string result;
        std::ostringstream id;
        id << cameraId;
        quad.putVariable("quad", ep);
        result = "direction = (quad * rodrigues(rotMatCamCoord_" + id.str() + "))'";
        engEvalString(ep, result.c_str());
        engEvalString(ep, "direction = rotationMatrix * direction");
    } else {
        engEvalString(ep, "direction = rotationMatrix * quad");
    }
    mxArray *d = engGetVariable(ep, "direction");
    direction = *(new Vector(mxGetPr(d)[0], mxGetPr(d)[1], mxGetPr(d)[2]));
    mxDestroyArray(d);

    // controlling whether all cameras already tracked the quadcopter once
    int valid = 0;
    for (int i = 0; i < numberCameras; i++) {
        if (quadPos[quadcopterId][i].isValid()) {
            valid++;
        }
    }
    if (valid != numberCameras) {
        // default value, when not all cameras tracked it yet
        // save result
        (quadPos[quadcopterId])[cameraId] = direction;
        ROS_DEBUG("Not all cameras did track quadcopter %d yet. Camera %d tracked it at position [%f, %f, %f]\n", quadcopterId, cameraId, direction.getV1(), direction.getV2(), direction.getV3());
        Vector nan = *(new Vector(NAN, NAN, NAN));
        return nan;
    } else {
        // save result
        (quadPos[quadcopterId])[cameraId] = direction;
        // not calculated before, first time calculating
        if (!(oldPos[quadcopterId].isValid())) {

            // building lines from camera position to quadcopter position
            Line *quadPositions = new Line[numberCameras];
            for (int i = 0; i < numberCameras; i++) {
                Vector camPos = getPosition(i);
                quadPositions[i] = *(new Line(camPos, quadPos[quadcopterId][i]));
            }

            Matlab *m = new Matlab(ep);
            Vector quadPosition = m->interpolateLines(quadPositions, numberCameras);

            oldPos[quadcopterId] = quadPosition;
            ROS_DEBUG("First seen position of quadcopter %d is [%f, %f, %f]\n", quadcopterId, quadPosition.getV1(), quadPosition.getV2(), quadPosition.getV3());

            return quadPosition;
        } else {
            Matlab *m = new Matlab(ep);

            // interpolation factor should be tested.
            Vector position = getPosition(cameraId);
            // line from camera to tracked object
            Line tracked = *(new Line(position, direction));
            // calulating actual pos
            Vector newPos = m->interpolateLine(tracked, oldPos[quadcopterId], 0.5);
            // saving new Pos
            ROS_DEBUG("New position of quadcopter %d is [%f, %f, %f]\n", quadcopterId, newPos.getV1(), newPos.getV2(), newPos.getV3());
            oldPos[quadcopterId] = newPos;
            return newPos;
        }
    }
}


Vector Position::getPositionInCameraCoordination(int cameraId) {
    Vector translation;
    if (cameraId == -1) {
        translation = *(new Vector(NAN, NAN, NAN));
    }
    else if (cameraId != 0) {
        loadValues(cameraId);
        std::string result;
        mxArray *tV = engGetVariable(ep, "T");
        std::ostringstream id;
        id << cameraId;
        result = "transVectCamCoord_" + id.str() + " = T";
        engEvalString(ep, result.c_str());
        translation = *(new Vector(mxGetPr(tV)[0], mxGetPr(tV)[1], mxGetPr(tV)[2]));
        mxDestroyArray(tV);
     } else {
        // camera 0 is at the origin and looks down the positive z axis
        translation = *(new Vector(0, 0, 0));
        engEvalString(ep, "transVectCamCoord_0 = [0, 0, 0]");
    }
    return translation;
}

Vector Position::getPosition(int cameraId) {
    Vector translation;
    if (cameraId == -1) {
        translation = *(new Vector(NAN, NAN, NAN));
    }
    else if (!(calibrated)) {
        if (cameraId == 0) {
            // camera 0 is at the origin and looks down the positive z axis
            translation = *(new Vector(0, 0, 0));
            Vector v = getCoordinationTransformation(translation,cameraId);
            std::string var;
            var = "cameraPosition_0";
            v.putVariable(var.c_str(), ep);
            translation = v;
        } else {
            translation = getPositionInCameraCoordination(cameraId);
            Vector v = getCoordinationTransformation(translation, cameraId);
            std::string var;
            std::ostringstream id;
            id << cameraId;
            var = "cameraPosition_" + id.str();
            id.str("");
            id.clear();
            v.putVariable(var.c_str(), ep);
            translation = v;
        }
     } else {
        std::string var;
        std::ostringstream id;
        id << cameraId;
        var = "cameraPosition_" + id.str();
        id.str("");
        id.clear();
        mxArray* trans = engGetVariable(ep, var.c_str());
        translation = *(new Vector(mxGetPr(trans)[0], mxGetPr(trans)[1], mxGetPr(trans)[2]));
        mxDestroyArray(trans);
    }
    return translation;
}

Vector Position::getOrientationInCameraCoordination(int cameraId) {
    Vector orientation;
    if (cameraId == -1) {
        orientation = *(new Vector(NAN, NAN, NAN));
    }
    else if (cameraId != 0) {
        loadValues(cameraId);
        std::string result;
        std::ostringstream id;
        id << cameraId;
        result = "rotMatCamCoord_" + id.str() + " = rodrigues(R);";
        engEvalString(ep, result.c_str());
        result = "rotMatCamCoord_" + id.str();
        mxArray *oV = engGetVariable(ep, result.c_str());
        orientation = *(new Vector(mxGetPr(oV)[0], mxGetPr(oV)[1], mxGetPr(oV)[2]));
        mxDestroyArray(oV);
    } else {
        // camera 0 is at the origin and looks down the positive z axis
        orientation = *(new Vector(0, 0, 1));
        engEvalString(ep, "rotMatCamCoord_0 = [0, 0, 1]");
    }
    return orientation;
}

Vector Position::getOrientation(int cameraId) {
    Vector orientation;
    if (cameraId == -1) {
        orientation = *(new Vector(NAN, NAN, NAN));
    }
    else {
        if (!(calibrated)) {
            if (cameraId == 0) {
                orientation = getOrientationInCameraCoordination(cameraId);
                // saving orientation in cameraOrientation_cameraIs
                Vector v = getCoordinationTransformation(orientation, cameraId);
                std::string var;
                var = "cameraOrientation_0";
                v.putVariable(var.c_str(), ep);
                orientation = v;
            }
            else {
                orientation = getOrientationInCameraCoordination(cameraId);
                // saving orientation in cameraOrientation_cameraIds
                Vector v = getCoordinationTransformation(orientation, cameraId);
                std::string var;
                std::ostringstream id;
                id << cameraId;
                var = "cameraOrientation_" + id.str();
                id.str("");
                id.clear();
                v.putVariable(var.c_str(), ep);
                orientation = v;
            }
        } else {
            std::string var;
            std::ostringstream id;
            id << cameraId;
            var = "cameraOrientation_" + id.str();
            id.str("");
            id.clear();
            mxArray* oV = engGetVariable(ep, var.c_str());
            orientation = *(new Vector(mxGetPr(oV)[0], mxGetPr(oV)[1], mxGetPr(oV)[2]));
            mxDestroyArray(oV);
        }
    }
    return orientation;
}

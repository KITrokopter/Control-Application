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
    calib = new AmccCalibration();
    Vector nan = *(new Vector(NAN, NAN, NAN));
    // if quadcopter maximal amount is higher than 50, you should change the range of i
    for (int i = 0; i < 50; i++) {
        std::vector<Vector> h(20, nan);
        quadPos.push_back(h);
        oldPos.push_back(nan);
    }
    this->transformed = false;

}

Position::Position(Engine *ep, int numberCameras)
{
    this->numberCameras = numberCameras;
    this->ep = ep;
    calib = new AmccCalibration(ep);
    Vector nan = *(new Vector(NAN, NAN, NAN));
    for (int i = 0; i < 50; i++) {
        std::vector<Vector> h(20, nan);
        quadPos.push_back(h);
        oldPos.push_back(nan);
    }
    this->transformed = false;
}

bool Position::calibrate(ChessboardData *chessboardData, int numberCameras) {

    this->numberCameras = numberCameras;
    if (numberCameras < 3) {
        ROS_ERROR("Not enough cameras!");
        return false;
    }

    calib->multiCameraCalibration(numberCameras, chessboardData->getChessFieldWidth(), chessboardData->getChessFieldHeight(), chessboardData->getNumberCornersX(), chessboardData->getNumberCornersY());

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
    }

    ROS_INFO("Finished multi camera calibration: %s",(ok)?"true":"false");

    Vector v0 = getPosition(0);
    Vector v1 = getPosition(1);
    Vector v2 = getPosition(2);

    ROS_DEBUG("Distance between camera 0 and 1 is %f", v0.add(v1.mult(-1)).getLength());
    ROS_DEBUG("Distance between camera 0 and 2 is %f", v0.add(v2.mult(-1)).getLength());
    ROS_DEBUG("Distance between camera 1 and 2 is %f", v1.add(v2.mult(-1)).getLength());

    return ok;
}

double Position::getAngle(Vector u, Vector v) {
    // cos(alpha)= u*v/(|u|*|v|)
    double angle = u.scalarMult(v)/(u.getLength() * v.getLength());
    return acos(angle);
}



// calculates Vector in the calibration coordination of camera 0 in the real camera coordination
Vector Position::getCoordinationTransformation(Vector w, int cameraId) {
    if (cameraId == -1) {
        Vector nan = *(new Vector(NAN, NAN, NAN));
        return nan;
    } else {
        if (transformed == false) {
            // Plain of the cameras E = a + r * u + s * (c - a)
            // as a is always the origin, E intersects the xy-plain in the origin => translation vector is not neccesary
            Matlab *m = new Matlab(ep);
            Vector a = getPositionInCameraCoordination(0);
            Vector b = getPositionInCameraCoordination(1);
            Vector c = getPositionInCameraCoordination(2);
            Vector u = b.add(a.mult(-1));
            Vector v = c.add(a.mult(-1));

            Line cameras = *(new Line(a, u));
            //printf("camera 1: [%f, %f, %f]\n", b.getV1(), b.getV2(), b.getV3());
            //printf("camera 2: [%f, %f, %f]\n", c.getV1(), c.getV2(), c.getV3());
            // calculates intersection line of plain of cameras in reality and plain of cameras in coordination system

            Vector origin = *(new Vector(1, 1, 0));
            Vector x = *(new Vector(-1, 0, 0));
            // as the method calculates y - origin = (0, -1, 0)
            Vector y = *(new Vector(1, 0, 0));
            Line xAxis = *(new Line(origin, x));
            // works if E isn't already on the x axis or the y axis
            Line intersectionLine = m->getIntersectionLine(cameras, c, xAxis, y);
            //printf("[%f, %f, %f] + r * [%f, %f, %f]\n", intersectionLine.getA().getV1(), intersectionLine.getA().getV2(), intersectionLine.getA().getV3(), intersectionLine.getU().getV1(), intersectionLine.getU().getV2(), intersectionLine.getU().getV3());

            // calculating angel xAxis and translation
            double angle = getAngle(x, intersectionLine.getU());

            // enter angle in matlab
            double dataAngleZ[1] = {-angle};
            mxArray *angZ = mxCreateDoubleMatrix(1, 1, mxREAL);
            memcpy((void *)mxGetPr(angZ), (void *)dataAngleZ, sizeof(dataAngleZ));
            engPutVariable(ep, "angleZ", angZ);
            mxDestroyArray(angZ);

            // calculating rotationmatrix of z axis
            engEvalString(ep, "rotationMatrixZ = [cos(angleZ) -sin(angleZ) 0; sin(angleZ) cos(angleZ) 0; 0 0 1];");

            // calculating angle between xy-plain and E (plain of cameras)
            // normal vector of xy plain
            Vector nxy = *(new Vector(0, 0, 1));
            // normal Vector of E
            Vector ncam = v.cross(u);

            angle = getAngle(nxy, ncam);

            // enter angle in matlab
            double dataAngleX[1] = {angle};
            mxArray *angX = mxCreateDoubleMatrix(1, 1, mxREAL);
            memcpy((void *)mxGetPr(angX), (void *)dataAngleX, sizeof(dataAngleX));
            engPutVariable(ep, "angleX", angX);
            mxDestroyArray(angX);

            // calculating rotationmatrix of x axis
            engEvalString(ep, "rotationMatrixX = [1 0 0; 0 cos(angleX) -sin(angleX); 0 sin(angleX) cos(angleX)];");


            /*
             * rotationMatrixX and rotationMatrixZ should be saved in matlab, so they don't need to be calculated all the time.
             */
            this->transformed = true;
       }

        w.putVariable("cameraCoordinationPos", ep);
        // calculate rotationMatrixX * rotationMatrixZ * vector in camera system 0
        engEvalString(ep, "result = rotationMatrixX * rotationMatrixZ * cameraCoordinationPos';");
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
    if (cameraId == 0) {
        std::string result = "load('/tmp/calibrationResult/Calib_Results_0.mat');";
        // loads resulting file in matlab workspace
        engEvalString(ep, result.c_str());
    } else {
        std::string result;
        std::ostringstream id;
        id << cameraId;
        result = "load('/tmp/calibrationResult/Calib_Results_stereo_0_" + id.str() + ".mat');";
        // loads resulting file in matlab workspace
        engEvalString(ep, result.c_str());
    }
}

Vector Position::updatePosition(Vector quad, int cameraId, double quadcopterId) {
    Vector pos;
    ROS_DEBUG("camera id is %d", cameraId);
    if (cameraId == -1) {
        Vector nan = *(new Vector(NAN, NAN, NAN));
        return nan;
    }
    else if (cameraId != 0) {

        //// SHOULDN'T BE LOADED ALL THE TIME!!!

        loadValues(cameraId);
        quad.putVariable("quad", ep);
        engEvalString(ep, "pos = (quad * rodrigues(omc_left_1))' + Tc_left_1;");
        mxArray *position = engGetVariable(ep, "pos");
        pos = *(new Vector(mxGetPr(position)[0], mxGetPr(position)[1], mxGetPr(position)[2]));
        pos = getCoordinationTransformation(pos, cameraId);
        mxDestroyArray(position);
    } else {
        ROS_DEBUG("coordination transformation beginning with camera id %d\n", cameraId);
        pos = getCoordinationTransformation(quad, cameraId);
        ROS_DEBUG("coordination transformation finished\n");
    }

    // controlling whether all cameras already tracked the quadcopter once
   /// if (quadPos[quadcopterId].size() < numberCameras) {
    int valid = 0;
    for (int i = 0; i < numberCameras; i++) {
        if (quadPos[quadcopterId][i].isValid()) {
            valid++;
        }
    }
    if (valid != numberCameras) {
        /// default value, when not all cameras did track it
        // save result
        (quadPos[quadcopterId])[cameraId] = pos;
        Vector nan = *(new Vector(NAN, NAN, NAN));
        return nan;
    } else {
        // save result
        (quadPos[quadcopterId])[cameraId] = pos;
        // not calculated before, first time
        if (!(oldPos[quadcopterId].isValid())) {

            // building lines from camera position to quadcopter position
            Line *quadPositions = new Line[numberCameras];
            for (int i = 0; i < numberCameras; i++) {
                Vector position = getPosition(i);
                quadPositions[i] = *(new Line(position, (quadPos[quadcopterId])[i].add(position.mult(-1))));
            }

            Matlab *m = new Matlab(ep);
            Vector quadPosition = m->interpolateLines(quadPositions, numberCameras);

            oldPos[quadcopterId] = quadPosition;

            //return nan, as no earlier position is known, so the movement can't be calculated
            Vector nan = *(new Vector(NAN, NAN, NAN));
            return nan;
        } else {
            Matlab *m = new Matlab(ep);
            // interpolation factor has to be tested.
            Vector position = getPosition(cameraId);
            // line from camera to tracked object
            Line tracked = *(new Line(position, pos.add(position.mult(-1))));
            // calulating actual pos
            Vector newPos = m->interpolateLine(tracked, oldPos[quadcopterId], 0.5);
            // calculating movement = newPos - oldPos
            Vector movement = newPos.add(oldPos[quadcopterId].mult(-1));
            // saving new Pos
            oldPos[quadcopterId] = newPos;
            return movement;
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
        mxArray *tV = engGetVariable(ep, "Tc_left_1");
        translation = *(new Vector(mxGetPr(tV)[0], mxGetPr(tV)[1], mxGetPr(tV)[2]));
     } else {
        // camera 0 is at the origin and looks down the positive z axis
        translation = *(new Vector(0, 0, 0));
    }
    return translation;
}

Vector Position::getPosition(int cameraId) {
    Vector translation;
    if (cameraId == -1) {
        translation = *(new Vector(NAN, NAN, NAN));
    }
    else if (!(transformed)) {
        if (cameraId == 0) {
            // camera 0 is at the origin and looks down the positive z axis
            translation = *(new Vector(0, 0, 0));
            Vector v = getCoordinationTransformation(translation,cameraId);
            std::string var;
            std::ostringstream id;
            id << cameraId;
            var = "cameraPosition_" + id.str();
            id.str("");
            id.clear();
            v.putVariable(var.c_str(), ep);
            translation = v;
        } else {
            loadValues(cameraId);
            mxArray *tV = engGetVariable(ep, "Tc_left_1");
            translation = *(new Vector(mxGetPr(tV)[0], mxGetPr(tV)[1], mxGetPr(tV)[2]));
            Vector v = getCoordinationTransformation(translation, cameraId);
            std::string var;
            std::ostringstream id;
            id << cameraId;
            var = "cameraPosition_" + id.str();
            id.str("");
            id.clear();
            v.putVariable(var.c_str(), ep);
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
        mxArray *oV = engGetVariable(ep, "omc_left_1");
        orientation = *(new Vector(mxGetPr(oV)[0], mxGetPr(oV)[1], mxGetPr(oV)[2]));
        mxDestroyArray(oV);
    } else {
        // camera 0 is at the origin and looks down the positive z axis
        orientation = *(new Vector(0, 0, 1));
    }
    return orientation;
}

Vector Position::getOrientation(int cameraId) {
    Vector orientation;
    if (cameraId != -1) {
        orientation = *(new Vector(NAN, NAN, NAN));
    }
    else {
        if (!(transformed)) {
            if (cameraId == 0) {
                // camera 0 is at the origin and looks down the positive z axis
                orientation = *(new Vector(0, 0, 1));
                // saving orientation in cameraOrientation_cameraIs
                Vector v = getCoordinationTransformation(v, cameraId);
                std::string var;
                std::ostringstream id;
                id << cameraId;
                var = "cameraOrientation_" + id.str();
                id.str("");
                id.clear();
                v.putVariable(var.c_str(), ep);
                orientation = v;
            }
            else {
                loadValues(cameraId);
                mxArray *oV = engGetVariable(ep, "omc_left_1");
                orientation = *(new Vector(mxGetPr(oV)[0], mxGetPr(oV)[1], mxGetPr(oV)[2]));
                mxDestroyArray(oV);
                // saving orientation in cameraOrientation_cameraIs
                Vector v = getCoordinationTransformation(v, cameraId);
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

#include "PositionCalculator.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <sstream>
#include <cstring>
#include <engine.h>
#include "Vector.h"
#include "Line.h"
#include "Matrix.h"
#include "TrackingMath.h"
#include "TrackingArea.h"
#include "AmccCalibration.h"
#include <vector>
#include <cmath>
#include <math.h>
#include <ros/ros.h>
#include "profiling.hpp"

PositionCalculator::PositionCalculator()
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

PositionCalculator::PositionCalculator(Engine *ep, int numberCameras)
{
	this->numberCameras = numberCameras;
	this->ep = ep;
	initialize();
}

void PositionCalculator::initialize()
{
	this->transformed = false;
	this->interpolationDependent = true;
	distance = 0;
	Matrix nanMatrix = Matrix(NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN);

	rotationMatrix = nanMatrix;
	if (interpolationDependent) {
		ROS_INFO("Using adaptive interpolation factor.");
	} else {
		ROS_INFO("Using always the same interpolation factor.");
	}
	m = TrackingMath();
}

PositionCalculator::~PositionCalculator()
{
	engClose(ep);
}

bool PositionCalculator::calibratedYet(int numberCameras)
{
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
		ROS_DEBUG("Camera 0 isn't calibrated yet.");
		ok = false;
	} else {
		for (int i = 1; i < numberCameras; i++) {
			id << i;
			load = "try load('/tmp/calibrationResult/Calib_Results_stereo_" + id.str() +
			       "_0.mat'); worked = 1; catch worked = 0; end";
			engEvalString(ep,load.c_str());
			good = engGetVariable(ep, "worked");
			result = mxGetPr(good)[0];

			if (result == 0) {
				ROS_DEBUG("Camera %d isn't calibrated yet", i);
				ok = false;
			}

			id.str("");
			id.clear();
		}
	}
	return ok;
}

bool PositionCalculator::calibrate(ChessboardData *chessboardData, int numberCameras)
{
	this->numberCameras = numberCameras;
	if (numberCameras < 3) {
		ROS_ERROR("Not enough cameras!");
		return false;
	}

	bool calculated = calibratedYet(numberCameras);

	if (calculated == false) {
		ROS_DEBUG("Calibrating with amcc toolbox. This may take up to 20 minutes.");
		AmccCalibration calib(ep);
		calib.multiCameraCalibration(numberCameras,
		                             chessboardData->getChessFieldWidth(),
		                             chessboardData->getChessFieldHeight(),
		                             chessboardData->getNumberCornersX(), chessboardData->getNumberCornersY());
	}

	// checking whether calibration did work (trying to load all output files)
	bool ok = calibratedYet(numberCameras);

	// saves all position and orientation vectors
	if (ok) {
		Vector invalid = Vector(false);
		Matrix invalidMatrix = Matrix(false);
		std::vector<Vector> h(numberCameras, invalid);

		// if quadcopter maximal amount is higher than 50, you should change the
		// range of i
		for (int i = 0; i < 50; i++) {
			quadPos.push_back(h);
			oldPos.push_back(invalid);
		}

		for (int i = 0; i < numberCameras; i++) {
			camCoordCameraPos.push_back(invalid);
			camCoordCameraOrient.push_back(invalid);
			camRotMat.push_back(invalidMatrix);
			realCameraPos.push_back(invalid);
			realCameraOrient.push_back(invalid);
			imageAge.push_back(0);
		}

		// calculates from the back as for camera 0 the rotationmatrix isn't
		// calculated yet
		for (int i = (numberCameras - 1); i >= 0; i--) {
			calculatePosition(i);
			calculateOrientation(i);
			if (!realCameraPos[i].isValid() || !realCameraOrient[i].isValid()) {
				ROS_ERROR("Amcc toolbox calculation didn't work!");
				exit(1);
			}
		}

		Vector v0 = realCameraPos[0];
		Vector d0 = realCameraOrient[0];
		Vector v1 = realCameraPos[1];
		Vector d1 = realCameraOrient[1];
		Vector v2 = realCameraPos[2];
		Vector d2 = realCameraOrient[2];

		ROS_DEBUG("Position camera 0: [%.2f, %.2f, %.2f] is directed in [%.3f, %.3f, %.3f]", v0.getV1(),
		          v0.getV2(), v0.getV3(), d0.getV1(), d0.getV2(), d0.getV3());
		ROS_DEBUG("Position camera 1: [%.2f, %.2f, %.2f] is directed in [%.3f, %.3f, %.3f]", v1.getV1(),
		          v1.getV2(), v1.getV3(), d1.getV1(), d1.getV2(), d1.getV3());
		ROS_DEBUG("Position camera 2: [%.2f, %.2f, %.2f] is directed in [%.3f, %.3f, %.3f]", v2.getV1(),
		          v2.getV2(), v2.getV3(), d2.getV1(), d2.getV2(), d2.getV3());

		ROS_DEBUG("Distance between camera 0 and 1 is %.2f", v0.add(v1.mult(-1)).getLength());
		ROS_DEBUG("Distance between camera 0 and 2 is %.2f", v0.add(v2.mult(-1)).getLength());
		ROS_DEBUG("Distance between camera 1 and 2 is %.2f", v1.add(v2.mult(-1)).getLength());

		setTrackingArea(2600);
		tracking.printTrackingArea();
	}

	ROS_INFO("Finished multi camera calibration: %s",(ok) ? "true" : "false");

	return ok;
}

void PositionCalculator::angleTry(int sign)
{
	// Plain of the cameras E = a + r * u + s * (c - a)
	// as a is always the origin, E intersects the xy-plain in the origin =>
	// translation vector is not neccesary

	// a, b, c are the positions of camera 0, 1, 2 in camera coordinate system 0
	// (might not be calculated yet)
	Vector a = Vector(0, 0, 0);
	loadValues(1);
	mxArray *r = engGetVariable(ep, "T");
	Vector b = Vector(mxGetPr(r)[0], mxGetPr(r)[1], mxGetPr(r)[2]);
	loadValues(2);
	r = engGetVariable(ep, "T");
	Vector c = Vector(mxGetPr(r)[0], mxGetPr(r)[1], mxGetPr(r)[2]);
	mxDestroyArray(r);
	Vector u = b.add(a.mult(-1));

	Line cameras = Line(a, u);
	// calculates intersection line of plain of cameras in reality and plane of
	// cameras in coordination system

	Vector origin = Vector(1, 1, 0);
	Vector x = Vector(-1, 0, 0);
	// as the method calculates y - origin = (0, -1, 0)
	Vector y = Vector(1, 0, 0);
	Line xAxis = Line(origin, x);
	// works if E isn't already on the x axis or the y axis
	Line intersectionLine = m.getIntersectionLine(cameras, c, xAxis, y);
	// ROS_DEBUG("[%f, %f, %f] + r * [%f, %f, %f]\n",
	// intersectionLine.getA().getV1(), intersectionLine.getA().getV2(),
	// intersectionLine.getA().getV3(), intersectionLine.getU().getV1(),
	// intersectionLine.getU().getV2(), intersectionLine.getU().getV3());

	Vector n = intersectionLine.getU().mult(1 / intersectionLine.getU().getLength());

	n.putVariable("n", ep);
	double angle = m.getAngle(Vector(0, 0, 1), b.cross(c));

	double dataAngle[1] = {sign *angle};
	mxArray *ang = mxCreateDoubleMatrix(1, 1, mxREAL);
	memcpy((void*)mxGetPr(ang), (void*)dataAngle, sizeof(dataAngle));
	engPutVariable(ep, "a", ang);
	engEvalString(ep,
	              "rotationMatrix = [(n(1)^2*(1-cos(a)) + cos(a)), (n(1)*n(2)*(1-cos(a))-n(3) * sin(a)), (n(1)*n(3)*(1-cos(a)) + n(2)*sin(a)); (n(2)*n(1)*(1-cos(a)) + n(3)*sin(a)), (n(2)^2*(1 - cos(a)) + cos(a)), (n(2) * n(3) * (1-cos(a)) - n(1) * sin(a)); (n(3) * n(1) *(1-cos(a)) - n(2) * sin(a)), (n(3) * n(2) * (1 - cos(a)) + n(1) * sin(a)), (n(3)^2 * (1-cos(a)) + cos(a))]");
}

// calculates Vector in the calibration coordinate of camera 0 in the real
// camera coordination
Vector PositionCalculator::calculateCoordinateTransformation(Vector w)
{
	if (transformed == false) {
		angleTry(1);

		// checking, whether the result of the z value is nearly 0, if you
		// rotate the first camera in coordinate system of camera 0
		loadValues(1);
		mxArray *r = engGetVariable(ep, "T");
		Vector firstCam = Vector(mxGetPr(r)[0], mxGetPr(r)[1], mxGetPr(r)[2]);
		r = engGetVariable(ep, "rotationMatrix");
		rotationMatrix = Matrix(mxGetPr(r)[0], mxGetPr(r)[3], mxGetPr(r)[6], mxGetPr(r)[1], mxGetPr(r)[4], mxGetPr(
		                            r)[7], mxGetPr(r)[2], mxGetPr(r)[5], mxGetPr(r)[8]);

		Vector result = firstCam.aftermult(rotationMatrix);

		if (!((result.getV3() < 0.5) && (result.getV3() > -0.5))) {
			ROS_DEBUG("first calculation of transformation matrix, camera 1 would be at position [%.2f, %.2f, %.2f]",
			          result.getV1(), result.getV2(), result.getV3());
			// if value is wrong, the angle has to be negativ
			angleTry(-1);
			r = engGetVariable(ep, "rotationMatrix");
			rotationMatrix = Matrix(mxGetPr(r)[0], mxGetPr(r)[3], mxGetPr(r)[6], mxGetPr(r)[1], mxGetPr(r)[4], mxGetPr(
			                            r)[7], mxGetPr(r)[2], mxGetPr(r)[5], mxGetPr(r)[8]);

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

void PositionCalculator::setNumberCameras(int numberCameras)
{
	this->numberCameras = numberCameras;
}

void PositionCalculator::loadValues(int cameraId)
{
	if (cameraId == 0) {
		// loads resulting file in matlab workspace
		engEvalString(ep, "load('/tmp/calibrationResult/Calib_Results_0.mat');");
	} else {
		std::string result;
		std::ostringstream id;
		id << cameraId;
		result = "load('/tmp/calibrationResult/Calib_Results_stereo_" + id.str() + "_0.mat');";
		// loads resulting file in matlab workspace
		engEvalString(ep, result.c_str());
	}
}

Vector PositionCalculator::updatePosition(std::vector<CameraData> cameraLines)
{
	int quadcopterId = cameraLines[0].quadcopterId;

	if (cameraLines.size() == 2) {
		ROS_DEBUG("POSITION_MODULE: Camera %d, %d see quadcopter", cameraLines[0].camNo, cameraLines[1].camNo);
	} else if (cameraLines.size() == 3) {
		ROS_DEBUG("POSITION_MODULE: Camera %d, %d, %d see quadcopter", cameraLines[0].camNo, cameraLines[1].camNo,
		          cameraLines[2].camNo);
	} else if (cameraLines.size() == 1) {
		ROS_DEBUG("POSITION_MODULE: Camera %d sees quadcopter", cameraLines[0].camNo);
	}


	for (int i = 1; i < cameraLines.size(); i++) {
		if (cameraLines[i].quadcopterId != quadcopterId) {
			ROS_ERROR("scheduler passes datas of different quadcopter ids!");
		}
	}

	// increments counter for other cameras
	for (int i = 0; i < numberCameras; i++) {
		bool tracked = false;
		for (int j = 0; j < cameraLines.size(); j++) {
			if (i == cameraLines[j].camNo) {
				tracked = true;
			}
		}
		if (!(tracked)) {
			imageAge[i]++;
		}
	}
	// resets counter of cameras that are tracking quadcopter
	for (int i = 0; i < cameraLines.size(); i++) {
		imageAge[cameraLines[i].camNo] = 0;
	}

	Vector direction[cameraLines.size()];

	// rotating coordinate system in coordinate system of camera 0 and then in
	// real coordination system
	// direction = rotationMatrix * (camRotMat * quad)
	for (int i = 0; i < cameraLines.size(); i++) {
		direction[i] =
		    (cameraLines[i].cameraVector.aftermult(camRotMat[cameraLines[i].camNo])).aftermult(rotationMatrix);
		// save result
		(quadPos[quadcopterId])[cameraLines[i].camNo] = direction[i];
	}

	// controlling whether all cameras already tracked the quadcopter once
	int valid = 0;
	for (int i = 0; i < numberCameras; i++) {
		if (quadPos[quadcopterId][i].getValid()) {
			valid++;
		}
	}

	if (valid != numberCameras) {
		// default value, when not all cameras tracked it yet
		// ROS_DEBUG("Not all cameras did track quadcopter %d yet.",
		// quadcopterId);
		Vector nan = Vector(NAN, NAN, NAN);
		return nan;
	} else {
		if (!(oldPos[quadcopterId].getValid())) {
			// not calculated before, first time calculating
			int tooOld = 0;
			for (int i = 0; i < numberCameras; i++) {
				if (imageAge[i] > 5) {
					// ROS_DEBUG("Information of camera %d is too old.", i);
					tooOld++;
				}
			}
			if (2 > numberCameras - tooOld) {
				// ROS_DEBUG("Information can't be used, as too much cameras
				// can't track it anymore");
				return Vector(NAN, NAN, NAN);
			}

			// building lines from camera position to quadcopter position
			Line quadPositions[numberCameras];
			for (int i = 0; i < numberCameras; i++) {
				Vector camPos = getPosition(i);
				quadPositions[i] = Line(camPos, quadPos[quadcopterId][i]);
			}

			Vector quadPosition = m.interpolateLines(quadPositions, numberCameras, Vector(0, 0, 0), 1);

			oldPos[quadcopterId] = quadPosition;
			ROS_INFO("POSITION_MODULE: First seen position of quadcopter %d is [%.2f, %.2f, %.2f], %s", quadcopterId,
			         quadPosition.getV1(), quadPosition.getV2(), quadPosition.getV3(), tracking.inCameraRange(
			             quadPosition) ? "in tracking area" : "NOT in tracking area");

			// as distance of 150 has interpolation factor 0.5
			distance = 150;
			return quadPosition;
		} else {
			// calculated before

			int tooOld = 0;
			for (int i = 0; i < numberCameras; i++) {
				if (imageAge[i] > 10) {
					// ROS_DEBUG("Information of camera %d is too old.", i);
					tooOld++;
				}
			}

			if (2 > numberCameras - tooOld) {
				ROS_WARN("POSITION_MODULE: Only camera %d still tracks quadcopter %d.", cameraLines[0].camNo,
				         cameraLines[0].quadcopterId);
			}

			Vector newPos;
			double interpolationFactor;

			if (interpolationDependent) {
				if (distance > 200) {
					interpolationFactor = 0.7;
				} else if (distance < 100) {
					interpolationFactor = 0.2;
				} else {
					// diff is between 0 and 100
					double diff = distance - 100;
					// diff is between 0.2 and 0.7
					interpolationFactor = 0.2 + 0.5 * diff / 100.0;
				}
			} else {
				interpolationFactor = 0.5;
			}

			if (cameraLines.size() > 1) {
				Line tracking[cameraLines.size()];
				for (int i = 0; i < cameraLines.size(); i++) {
					tracking[i] =
					    Line(getPosition(cameraLines[i].camNo), quadPos[quadcopterId][(cameraLines[i].camNo)]);
					// ROS_DEBUG("camera %d, [%f, %f, %f] + r * [%f, %f, %f]",
					// cameraLines[i].camNo,
					// getPosition(cameraLines[i].camNo).getV1(),
					// getPosition(cameraLines[i].camNo).getV2(),
					// getPosition(cameraLines[i].camNo).getV3(),
					// quadPos[quadcopterId][(cameraLines[i].camNo)].getV1(),
					// quadPos[quadcopterId][(cameraLines[i].camNo)].getV2(),
					// quadPos[quadcopterId][(cameraLines[i].camNo)].getV3());
				}
				newPos = m.interpolateLines(tracking, cameraLines.size(), oldPos[quadcopterId], interpolationFactor);
			} else {
				Vector position = getPosition(cameraLines[0].camNo);
				Line tracked = Line(position, quadPos[quadcopterId][0]);

				newPos = m.interpolateLine(tracked, oldPos[quadcopterId], interpolationFactor);
			}
			if (newPos.getValid()) {
				this->error = m.getError();

				// calculates distance between last seen position and new
				// calculated position
				distance = (oldPos[quadcopterId]).add(newPos.mult(-1)).getLength();

				ROS_DEBUG("POSITION_MODULE: error is %.2f, distance is %.2f", error, distance);

				// saving new Pos
				bool trackingArea = tracking.inCameraRange(newPos);
				if (trackingArea) {
					ROS_INFO("POSITION_MODULE: New position of quadcopter %d is [%.2f, %.2f, %.2f], %s", quadcopterId,
					         newPos.getV1(), newPos.getV2(), newPos.getV3(), "in tracking area");
				} else {
					if (newPos.add(realCameraPos[0].mult(-1)).getLength() > 2300) {
						ROS_INFO(
						    "POSITION_MODULE: New position of quadcopter %d is [%.2f, %.2f, %.2f], %s too far of camera 0", quadcopterId,
						    newPos.getV1(), newPos.getV2(), newPos.getV3(), "NOT in tracking area");
					}
					if (newPos.add(realCameraPos[1].mult(-1)).getLength() > 2300) {
						ROS_INFO(
						    "POSITION_MODULE: New position of quadcopter %d is [%.2f, %.2f, %.2f], %s too far of camera 1", quadcopterId,
						    newPos.getV1(), newPos.getV2(), newPos.getV3(), "NOT in tracking area");
					}
					if (newPos.add(realCameraPos[2].mult(-1)).getLength() > 2300) {
						ROS_INFO(
						    "POSITION_MODULE: New position of quadcopter %d is [%.2f, %.2f, %.2f], %s too far of camera 2", quadcopterId,
						    newPos.getV1(), newPos.getV2(), newPos.getV3(), "NOT in tracking area");
					} else {
						ROS_INFO("POSITION_MODULE: New position of quadcopter %d is [%.2f, %.2f, %.2f], %s",
						         quadcopterId, newPos.getV1(), newPos.getV2(), newPos.getV3(), "NOT in tracking area");
					}
				}
				oldPos[quadcopterId] = newPos;
			} else {
				ROS_WARN("Couldn't calculate new position as angle between camera lines is too small");
			}

			return newPos;
		}
	}
}

void PositionCalculator::calculatePosition(int cameraId)
{
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

Vector PositionCalculator::getPosition(int cameraId)
{
	return this->realCameraPos[cameraId];
}

void PositionCalculator::calculateOrientation(int cameraId)
{
	if (cameraId != -1) {
		if (cameraId != 0) {
			loadValues(cameraId);
			mxArray *r = engGetVariable(ep, "R");
			camRotMat[cameraId] = Matrix(mxGetPr(r)[0], mxGetPr(r)[3], mxGetPr(r)[6], mxGetPr(r)[1], mxGetPr(
			                                 r)[4], mxGetPr(r)[7], mxGetPr(r)[2], mxGetPr(r)[5], mxGetPr(r)[8]);
			camCoordCameraOrient[cameraId] = (Vector(0, 0, 1)).aftermult(camRotMat[cameraId]);
		} else {
			// camera 0 is at the origin and looks down the positive z axis
			camRotMat[0] = Matrix(1, 0, 0, 0, 1, 0, 0, 0, 1);
			camCoordCameraOrient[0] = Vector(0, 0, 1);
		}
		realCameraOrient[cameraId] = camCoordCameraOrient[cameraId].aftermult(rotationMatrix);
	}
}

void PositionCalculator::setTrackingArea(double maxRange)
{
	this->tracking = TrackingArea(realCameraPos, realCameraOrient, numberCameras, maxRange);
}

TrackingArea PositionCalculator::getTrackingArea()
{
	return this->tracking;
}

double PositionCalculator::getDistance()
{
	return this->distance;
}

cv::Mat PositionCalculator::getDistortionCoefficients(int cameraId)
{
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

cv::Mat PositionCalculator::getIntrinsicsMatrix(int cameraId)
{
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

Matrix PositionCalculator::getRotationMatrix(int cameraId)
{
	Matrix rotation = rotationMatrix.multiplicate(camRotMat[cameraId]);
	return rotation;
}

double PositionCalculator::getError()
{
	return this->error;
}


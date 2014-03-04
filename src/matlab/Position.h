#ifndef POSITION_H
#define POSITION_H
#include "engine.h"
#include "AmccCalibration.h"
#include "../position/ChessboardData.hpp"
#include "Vector.h"
#include "Line.h"
#include "Matlab.h"
#include "AmccCalibration.h"
#include "engine.h"
#include <vector>

class Position {
private: 
	Engine *ep;
	AmccCalibration *calib;
	int numberCameras;
	// saves whether the cameras are calibrated or not;
	bool transformed;
	/// (quadPos[i])[j] is the position of quadrocopter with ID i and camera with ID j
	std::vector< std::vector <Vector> > quadPos;
	std::vector<Vector> oldPos;

	/* 
		Variables in matlab:
		cameraPosition_cameraId: real position of camera with cameraId
		cameraOrientation_cameraId: real orientation of camera with cameraId
		rotationMatrixX: rotationmatrix of x axis to calculated camera system of camera 0 to real coordination system
		rotationMatrixZ: rotationmatrix of z axis to calculated camera system of camera 0 to real coordination system

		camera values in camera coordination system with cameraId
		rotMatCamCoord_cameraId: rodrigues(omc_left_x)
		transVectCamCoord_cameraId


		camera values in real coordinationsystem
		rotationMatrixCamera_cameraId: rotationmatrix of camera with cameraId, is rodrigues(omc_left_x)
		translationVectorCamera_cameraId: translation vector of camera with cameraId, is Tc_left_1
	*/

public:
	// maximal amount of quadcopters is 50, maximal amount of cameras is 20
    	Position();
    	Position(Engine *ep, int numberCameras);
        bool calibrate(ChessboardData *chessboardData, int numberCameras);
	/// quad is vector of camera with cameraId, that points to quadcopter with quadcopterId, returns (Nan, NaN, NaN) the first time, the position is calculated, or if not all cameras did track it yet
	double getAngle(Vector u, Vector v);
	void setNumberCameras(int numberCameras);	
	Vector getCoordinationTransformation(Vector w, int cameraId);	
    	Vector updatePosition(Vector quad, int cameraId, double quadcopterId);
	/// should only be called once
        Vector getPositionInCameraCoordination(int cameraId);
	/// should only be called once        
	Vector getOrientationInCameraCoordination(int cameraId);
	/// should only be called once, value is saved in variable cameraPosition_cameraId
	Vector getPosition(int cameraId);
	/// should only be called once, value is saved in variable cameraOrientation_cameraId
	Vector getOrientation(int cameraId);
    // returns value of image, that is not suppressed
    int loadValues(int cameraId);
};

#endif // POSITION_H //

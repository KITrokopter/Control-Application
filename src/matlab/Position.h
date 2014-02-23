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
	AmccCalibration calib;
	int numberCameras;
	/// (quadPos[i])[j] is the position of quadrocopter with ID i and camera with ID j
	std::vector< std::vector <Vector> > quadPos;
	std::vector<Vector> oldPos;
public:
	// maximal amount of quadcopters is 50
    	Position();
    	Position(Engine *ep, int numberCameras);
        bool calibrate(ChessboardData *chessboardData, int numberCameras);
	/// quad is vector of camera with cameraId, that points to quadcopter with quadcopterId, returns (Nan, NaN, NaN) the first time, the position is calculated, or if not all cameras did track it yet
	double getAngle(Vector u, Vector v);
	Vector getCoordinationTransformation(Vector w, int cameraId);	
    	Vector updatePosition(Vector quad, int cameraId, double quadcopterId);
        Vector getPositionInCameraCoordination(int cameraId);
        Vector getOrientationInCameraCoordination(int cameraId);
	Vector getPosition(int cameraId);
	Vector getOrientation(int cameraId);
	void loadValues(int cameraId);
};

#endif // POSITION_H //

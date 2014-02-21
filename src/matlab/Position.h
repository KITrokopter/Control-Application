#ifndef POSITION_H
#define POSITION_H
#include "engine.h"
#include "AmccCalibration.h"
#include "../position/ChessboardData.hpp"
#include <vector>

class Position {
public:
    	Position();
    	Position(Engine *ep, int numberCameras);
    	void calibrate(ChessboardData *chessboardData, int cameraId);
	/// quad is vector of camera with cameraId, that points to quadcopter with quadcopterId    	
	Vector updatePosition(Vector quad, int cameraId, double quadcopterId);
    	Vector getPosition(int cameraId);
    	Vector getOrientation(int cameraId);
	void loadValues(int cameraId);
private: 
	Engine *ep;
	AmccCalibration calib;
	int numberCameras;
	/// (quadPos[i])[j] is the position of quadrocopter with ID i and camera with ID j
	std::vector< std::vector <Vector> > quadPos;
	std::vector<Vector> oldPos;
};

#endif // POSITION_H //

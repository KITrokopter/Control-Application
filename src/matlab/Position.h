#ifndef POSITION_H
#define POSITION_H
#include "engine.h"
#include "AmccCalibration.h"
#include "Transformation.h"
#include "../position/ChessboardData.hpp"

class Position {
public:
    Position();
    Position(Engine *ep);
    Transformation calibrate(ChessboardData *chessboardData, int cameraId);
    Vector updatePosition(Vector v, int cameraId, double quadcopterId);
    Vector getPosition(int cameraId);
    Vector getOrientation(int cameraId);
	
private: 
	Engine *ep;
	AmccCalibration calib;
	int numberCameras;
};

#endif // POSITION_H //?

#ifndef POSITION_H
#define POSITION_H
#include "engine.h"
#include "Calibration.h"
#include "../position/ChessboardData.hpp"

class Position {
private: Engine *ep;
	 Calibration calib;
public:
    Position(Engine *ep);
    void calibrate(ChessboardData *chessboardData, int cameraId);
    Vector updatePosition(Vector v, int cameraId, double quadcopterId);
    Vector getPosition(int cameraId);
    Vector getOrientation(int cameraId);
};

#endif // POSITION_H

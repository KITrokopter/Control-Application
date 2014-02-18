#ifndef POSITION_H
#define POSITION_H
#include "engine.h"
#include "Calibration.h"
#include "CameraData.h"

class Position {
private: Engine *ep;
	 Calibration calib;
public:
    Position(Engine *ep);
    void calibrate(CameraData cameraData, double cameraId);
    Vector updatePosition(Vector v, double cameraId, double quadcopterId);
    Vector getPosition(double cameraId);
    Vector getOrientation(double cameraId);
};

#endif // POSITION_H

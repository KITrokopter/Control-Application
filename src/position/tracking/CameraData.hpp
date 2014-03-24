#pragma once

#include "../../matlab/Vector.h"

class CameraData {
public:
	Vector cameraVector;
	int camNo;
	int quadcopterId;
	long int time;
	bool valid;
};
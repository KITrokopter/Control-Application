#pragma once

#include "../../src/position/IPositionReceiver.hpp"
#include "../../src/matlab/Vector.h"

class MovementGenerator {
private:
	double errorRate;
	
	Vector from;
	Vector to;
};
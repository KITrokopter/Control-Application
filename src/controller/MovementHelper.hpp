/*
 * MovementHelper.hpp
 *
 *  Created on: 30.03.2014
 *      Author: dwx
 */

#ifndef MOVEMENTHELPER_HPP_
#define MOVEMENTHELPER_HPP_

#include <list>
#include <cmath>

#include "Position6DOF.hpp"

/**
 * Class MovementHelper, currently unused.
 * It's intention was to prepare given position (through cameras) with
 * given rotation (through yawrate-sensor) to react with reasonable roll-/pitch-values.
 */
class MovementHelper {
public:
	MovementHelper();

	static Position6DOF prepareForRP(double rotation, Position6DOF pos, Position6DOF target);
};

#endif /* MOVEMENTHELPER_HPP_ */

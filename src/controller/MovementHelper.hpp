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

class MovementHelper {
public:
	MovementHelper();
//	double getHeightDiff( std::list<Position6DOF> listPositions );
//	double getHeightDiffReverse( std::list<Position6DOF> listPositions );

	static Position6DOF prepareForRP( double rotation, Position6DOF pos, Position6DOF target );
};

#endif /* MOVEMENTHELPER_HPP_ */

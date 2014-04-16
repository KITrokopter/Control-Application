/*
 * MovementHelper.cpp
 *
 *  Created on: 30.03.2014
 *      Author: dwx
 */

#include "MovementHelper.hpp"

MovementHelper::MovementHelper()
{
}

Position6DOF MovementHelper::prepareForRP(double rotation, Position6DOF pos, Position6DOF target)
{
	/* values of the Matrix, mij is value of i. line and j. column  */
	double m11 = cos(rotation);
	double m12 = -sin(rotation);
	double m21 = sin(rotation);
	double m22 = cos(rotation);
	double *currentPos = pos.getPosition();
	double *targetPos = target.getPosition();
	double newPosition[3];
	for (int i = 0; i < 3; i++) {
		currentPos[i] = currentPos[i] - targetPos[i];
	}
	newPosition[0] = m11 * currentPos[0] + m12 * currentPos[1];
	newPosition[1] = m21 * currentPos[0] + m22 * currentPos[1];
	newPosition[2] = currentPos[2];
	for (int i = 0; i < 3; i++) {
		currentPos[i] = currentPos[i] + targetPos[i];
	}

	pos.setPosition(newPosition);
	return pos;
}


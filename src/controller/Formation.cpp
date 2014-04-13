#include "Formation.hpp"
#include <stdlib.h>

/*
 * Constructor for formation with distance, number of crazyflies and structure
 * of the crazyflies
 */
Formation::Formation(int distance, int amount, Position6DOF *position)
{
	this->distance = distance;
	this->amount = amount;
	for (int i = 0; i < amount; i++) {
		this->position[i].setPosition(position[i].getPosition());
		this->position[i].setOrientation(position[i].getOrientation());
	}
}

/*
 * Empty Constructor
 */
Formation::Formation()
{
	this->amount = 0;
	this->distance = 0;
}

/*
 * Setter for Distance
 */
void Formation::setDistance(int distance)
{
	this->distance = distance;
}

/*
 * Setter for Amount of Crazyflies
 */
void Formation::setAmount(int amount)
{
	this->amount = amount;
}

/*
 * Setter of Formation Positions/Structure
 */
void Formation::setFormationPosition(Position6DOF *position)
{
	if (amount > 0) {
		for (int i = 0; i < amount; i++) {
			this->position[i].setPosition(position[i].getPosition());
			this->position[i].setOrientation(position[i].getOrientation());
		}
	}
}

/*
 * Getter for Distance
 */
int Formation::getDistance()
{
	return this->distance;
}

/*
 * Getter for Amount of Crazyflies
 */
int Formation::getAmount()
{
	return this->amount;
}

/*
 * Getter of Formation Positions/Structure
 */
Position6DOF* Formation::getFormationPosition()
{
	if (amount > 0) {
		return this->position;
	}
	return NULL;
}



#include "Formation.hpp"
//Don't know if we need that
Formation::Formation(int distance, int amount, Position6DOF * pos)
{
	this->distance = distance;
	this->amount = amount;
	for(int i = 0; i < amount; i++)
	{
		this->position[i].setPosition(position[i].getPosition());
		this->position[i].setOrientation(position[i].getOrientation());
	}
}

void Formation::setDistance(int distance)
{
	this->distance = distance;
}

void Formation::setAmount(int amount)
{
	this->amount = amount;
}

void Formation::setPosition(Position6DOF * position)
{
	for(int i=0; i < amount; i++)
	{
		this->position[i].setPosition(position[i].getPosition());
		this->position[i].setOrientation(position[i].getOrientation());
	}
}

int Formation::getDistance()
{
	return this->distance;
}

int Formation::getAmount()
{
	return this->amount;
}

Position6DOF * Formation::getPosition()
{
	return this->position;
}

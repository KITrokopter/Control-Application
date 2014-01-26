#include "Formation.hpp"

Formation::Formation(int distance, int amount, Position6DOF * position)
{
	this->distance = distance;
	this->amount = amount;
	for(int i=0; i < amount; i++)
	{
		this->position[i] = position[i];
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
		this->position[i] = position[i];
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


#include "Formation.hpp"
#include <stdlib.h>
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

Formation::Formation()
{
	/*TODO*/
	this->amount = 0;
	this->distance = 0;
}

void Formation::setDistance(int distance)
{
	this->distance = distance;
}

void Formation::setAmount(int amount)
{
	this->amount = amount;
	//Position6DOF *space = (Position6DOF*) malloc(sizeof(Position6DOF) * amount);
	//setPosition(space);
	//this->position = new Position6DOF[amount]();
	//this->position = (Position6DOF*) malloc(amount * sizeof(Position6DOF));
}

void Formation::setPosition(Position6DOF * position)
{
	ROS_INFO("Setting Formation Position");
	ROS_INFO("Amount is %i", amount);
	if(amount > 0)
	{
		for(int i=0; i < amount; i++)
		{
			ROS_INFO("Round %i", i);
			this->position[i].setPosition(position[i].getPosition());
			this->position[i].setOrientation(position[i].getOrientation());
		}
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
	if(amount > 0)
	{
		//(Position6DOF*) this->position;
		return this->position;
	}
	return NULL;
}

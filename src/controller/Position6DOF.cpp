
#include "Position6DOF.hpp"

Position6DOF::Position6DOF(int x, int y, int z, short int xOrientation, short int yOrientation, short int zOrientation)
{
	position[0] = (x);
	position[1] = (y);
	position[2] = (z);
	orientation[0] = (xOrientation);
	orientation[1] = (yOrientation);
	orientation[2] = (zOrientation);
}

Position6DOF::Position6DOF(int x, int y, int z)
{
	position[0] = (x);
	position[1] = (y);
	position[2] = (z);
	orientation[0] = 0;
	orientation[1] = 0;
	orientation[2] = 0;
}

int* Position6DOF::getPosition()
{
	return position;
}


short int* Position6DOF::getOrientation()
{
	return orientation;
}

void Position6DOF::setPosition(int* position)
{
	for(int i = 0; i < 3; i++)
	{	
		this->position[i] = position[i];
	}
	
}

void Position6DOF::setOrientation(short int* orientation)
{
	for(int i = 0; i < 3; i++)
	{	
		this->orientation[i] = orientation[i];
	}
}

time_t Position6DOF::getTimestamp()
{
	return timestamp;
}

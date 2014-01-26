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

int* Position6DOF::getPosition()
{
	return position;
}


short int* Position6DOF::getOrientation()
{
	return orientation;
}


time_t Position6DOF::getTimestamp()
{
	return timestamp;
}

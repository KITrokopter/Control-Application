
#include "Position6DOF.hpp"

Position6DOF::Position6DOF(double x, double y, double z,  double xOrientation,  double yOrientation,  double zOrientation)
{
	position[0] = (x);
	position[1] = (y);
	position[2] = (z);
	orientation[0] = (xOrientation);
	orientation[1] = (yOrientation);
	orientation[2] = (zOrientation);
}

Position6DOF::Position6DOF(double x, double y, double z)
{
	position[0] = (x);
	position[1] = (y);
	position[2] = (z);
	orientation[0] = 0;
	orientation[1] = 0;
	orientation[2] = 0;
}

double* const Position6DOF::getPosition()
{
	return position;
}


 double* const Position6DOF::getOrientation()
{
	return orientation;
}

void Position6DOF::setPosition(double* const position)
{
	for(int i = 0; i < 3; i++)
	{	
		this->position[i] = position[i];
	}
	
}

void Position6DOF::setOrientation( double* const orientation)
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

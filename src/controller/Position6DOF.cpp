
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

double* Position6DOF::getPosition()
{
	return position;
}


 double* Position6DOF::getOrientation()
{
	return orientation;
}

void Position6DOF::setPosition(double* position)
{
	for(int i = 0; i < 3; i++)
	{	
		this->position[i] = position[i];
	}
	
}

void Position6DOF::setOrientation( double* orientation)
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

void Position6DOF::setTimestamp(time_t newTimestamp) 
{
	this->timestamp = newTimestamp;
}


double Position6DOF::getAbsoluteDistance( Position6DOF otherPosition )
{
	double sum = 0;
	double distanceOfOne;
	for( int i = 0; i < 3; i++ )
	{
		distanceOfOne = position[i] - otherPosition.getPosition()[i];
		distanceOfOne *= distanceOfOne;
		sum += distanceOfOne;
	}
	return sqrt( sum );
}

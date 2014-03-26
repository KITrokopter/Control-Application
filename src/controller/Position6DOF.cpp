
#include "Position6DOF.hpp"

Position6DOF::Position6DOF(double x, double y, double z,  double xOrientation,  double yOrientation,  double zOrientation)
{
	this->position[0] = (x);
	this->position[1] = (y);
	this->position[2] = (z);
	this->orientation[0] = (xOrientation);
	this->orientation[1] = (yOrientation);
	this->orientation[2] = (zOrientation);
	this->timestamp = getNanoTime();
}

Position6DOF::Position6DOF(double x, double y, double z)
{
	this->position[0] = (x);
	this->position[1] = (y);
	this->position[2] = (z);
	this->orientation[0] = 0;
	this->orientation[1] = 0;
	this->orientation[2] = 0;
	this->timestamp = getNanoTime();
}

Position6DOF::Position6DOF(Vector vector)
{
	this->position[0] = vector.getV1();
	this->position[1] = vector.getV2();
	this->position[2] = vector.getV3();
	this->timestamp = getNanoTime();
}

double* Position6DOF::getPosition()
{
	return this->position;
}


 double* Position6DOF::getOrientation()
{
	return this->orientation;
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

long int Position6DOF::getTimestamp()
{
	return this->timestamp;
}

void Position6DOF::setTimestamp(long int newTimestamp) 
{
	this->timestamp = newTimestamp;
}


double Position6DOF::getAbsoluteDistance()
{
	double sum = 0;
	for( int i = 0; i < 3; i++ )
	{
		sum = sum + (this->position[i] * this->position[i]);
	}
	return sqrt( sum );
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

double Position6DOF::getDistanceZ( Position6DOF otherPosition )
{
	return (otherPosition.getPosition()[2] - this->position[2]);
}

void Position6DOF::predictNextPosition( Position6DOF olderPosition, long int timeInFuture )
{
	long int timediff = this->timestamp - olderPosition.getTimestamp();
	if(timediff == 0)
        {
                ROS_ERROR("this->timestamp %ld and olderPos %ld", this->timestamp, olderPosition.getTimestamp());
        }

	this->timestamp = this->timestamp + timeInFuture;

	double xDiff = this->position[0] - olderPosition.getPosition()[0];
	double yDiff = this->position[1] - olderPosition.getPosition()[1];
	double zDiff = this->position[2] - olderPosition.getPosition()[2];
	double rate;
	if(timediff == 0)
	{
		ROS_ERROR("Timediff is zero");
		rate = 0;
	}
	else
	{
		rate = ((double) timeInFuture) /((double) timediff) / 1000000000;
		ROS_INFO("rate %f", rate);
	}
	double xNew = this->position[0] + xDiff*rate;
	double yNew = this->position[1] + yDiff*rate;
	double zNew = this->position[2] + zDiff*rate;
	this->position[0] = xNew;
	this->position[1] = yNew;
	this->position[2] = zNew;
}

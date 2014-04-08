#include "Position6DOF.hpp"

/*
 * Constructor for Position6DOF with all position and orientation data
 */
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

/*
 * Constructor for Position6DOF with all position data
 */
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

/*
 * Constructor for Position6DOF with a vector for position
 */
Position6DOF::Position6DOF(Vector vector)
{
	this->position[0] = vector.getV1();
	this->position[1] = vector.getV2();
	this->position[2] = vector.getV3();
	this->timestamp = getNanoTime();
}

/*
 * Getter for Position
 */
double* Position6DOF::getPosition()
{
	return this->position;
}


/*
 * Getter for Orientation
 */
double* Position6DOF::getOrientation()
{
	return this->orientation;
}

/*
 * Setter for Position
 */
void Position6DOF::setPosition(double* position)
{
	for(int i = 0; i < 3; i++)
	{	
		this->position[i] = position[i];
	}
	
}

/*
 * Setter for orientation
 */
void Position6DOF::setOrientation( double* orientation)
{
	for(int i = 0; i < 3; i++)
	{	
		this->orientation[i] = orientation[i];
	}
}

/*
 * Getter for timestamp
 */
long int Position6DOF::getTimestamp()
{
	return this->timestamp;
}

/*
 * Setter for timestamp
 */
void Position6DOF::setTimestamp(long int newTimestamp) 
{
	this->timestamp = newTimestamp;
}


/*
 * Calculates Absolute Distance
 */
double Position6DOF::getAbsoluteDistance()
{
	double sum = 0;
	for( int i = 0; i < 3; i++ )
	{
		sum = sum + (this->position[i] * this->position[i]);
	}
	return sqrt( sum );
}

/*
 * Calculates Absolute Distance between this position and the position given 
 * as an argument.
 */
double Position6DOF::getAbsoluteDistance( Position6DOF otherPosition )
{
	double sum = 0;
	double distanceOfOne;
	for( int i = 0; i < 3; i++ )
	{
		distanceOfOne = abs( position[i] - otherPosition.getPosition()[i]);
		distanceOfOne *= distanceOfOne;
		sum += distanceOfOne;
	}
	return sqrt( sum );
}


/*
 * Calculates Absolute Distance between this position and the position given 
 * as an argument.
 */
double Position6DOF::getAbsoluteDistanceXY( Position6DOF otherPosition )
{
	double x = otherPosition.getPosition()[0] - this->position[0];
	x = x * x;
	double y = otherPosition.getPosition()[1] - this->position[1];
	y = y * y;
	return sqrt(x+y);
}

double Position6DOF::getDistanceX( Position6DOF otherPosition )
{
	return (otherPosition.getPosition()[0] - this->position[0]);
}

double Position6DOF::getDistanceY( Position6DOF otherPosition )
{
	return (otherPosition.getPosition()[1] - this->position[1]);
}

double Position6DOF::getDistanceZ( Position6DOF otherPosition )
{
	return (otherPosition.getPosition()[2] - this->position[2]);
}

double Position6DOF::getDistanceZ( Position6DOF otherPosition, double koeff )
{
	return (otherPosition.getPosition()[2] - (koeff - 1.0) / koeff * this->position[2]);
}

void Position6DOF::predictNextPosition( Position6DOF olderPosition, long int timeInFuture )
{
	long int timediff = this->timestamp - olderPosition.getTimestamp();
	if(timediff == 0)
	{
		ROS_ERROR("this->timestamp %ld and olderPos %ld", this->timestamp, olderPosition.getTimestamp());
	} 
	else
	{
		double timediffNorm = ((double) timediff) / ((double) 1000000000);
		//ROS_DEBUG("timediffNorm: %f", timediffNorm);
	}
	this->timestamp = this->timestamp + timeInFuture;

	double xDiff = this->position[0] - olderPosition.getPosition()[0];
	double yDiff = this->position[1] - olderPosition.getPosition()[1];
	double zDiff = this->position[2] - olderPosition.getPosition()[2];
	double rate = 0;
	if( timediff != 0 )
	{
		rate = (((double) timeInFuture) - ((double) timediff)) / 1000000000;
		//ROS_INFO("rate %f", rate);
	}
	
	double xNew = this->position[0] + xDiff*rate;
	double yNew = this->position[1] + yDiff*rate;
	double zNew = this->position[2] + zDiff*rate;
	this->position[0] = xNew;
	this->position[1] = yNew;
	this->position[2] = zNew;
}

#include "MovementQuadruple.hpp"

MovementQuadruple(int newThrust, float newRoll, float newPitch, float newYawrate)
{
	this->thrust = newThrust;
	this->roll = newRoll;
	this->pitch = newPitch;
	this->yawrate = newYawrate;		
}

void MovementQuadruple::setThrust( int newThrust )
{
	this->thrust = newThrust;
}

void MovementQuadruple::setRollPitchYawrate(float newRoll, float newPitch, float newYawrate)
{
	this->roll = newRoll;
	this->pitch = newPitch;
	this->yawrate = newYawrate;	
}

int MovementQuadruple::getThrust()
{
	return this->thrust;
}

float MovementQuadruple::getRoll()
{
	return this->roll;
}

float gMovementQuadruple::etPitch()
{
	return this->pitch;
}

float MovementQuadruple::getYawrate()
{
	return this->yawrate;
}



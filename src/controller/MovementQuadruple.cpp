#include "MovementQuadruple.hpp"

MovementQuadruple::MovementQuadruple(int newThrust, float newRoll, float newPitch, float newYawrate)
{
	this->thrust = newThrust;
	this->roll = newRoll;
	this->pitch = newPitch;
	this->yawrate = newYawrate;		
}

MovementQuadruple::MovementQuadruple(int newThrust, float newRoll, float newPitch, float newYawrate, time_t newTimestamp)
{
	this->thrust = newThrust;
	this->roll = newRoll;
	this->pitch = newPitch;
	this->yawrate = newYawrate;	
	this->timestamp = newTimestamp;		
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

float MovementQuadruple::getPitch()
{
	return this->pitch;
}

float MovementQuadruple::getYawrate()
{
	return this->yawrate;
}

long int MovementQuadruple::getTimestamp()
{
	return this->timestamp;
}

void MovementQuadruple::setTimestamp(long int newTimestamp) 
{
	this->timestamp = newTimestamp;
}
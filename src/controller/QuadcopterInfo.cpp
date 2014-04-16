#include "QuadcopterInfo.hpp"

QuadcopterInfo::QuadcopterInfo()
{
	this->state = UNSTARTED;
	// timeStarted;
	// timeLastUpdated;
	// timeShutdownStarted;
	x = 0.0;
	y = 0.0;
	rotation = 0.0;
	negativeSign = false;
}

/** Getter and Setter */
short QuadcopterInfo::getState()
{
	return this->state;
}

void QuadcopterInfo::setState(short newState)
{
	this->state = newState;
}

long int QuadcopterInfo::getStarted()
{
	return this->timeStarted;
}

void QuadcopterInfo::setStarted(long int newStarted)
{
	this->timeStarted = newStarted;
}

long int QuadcopterInfo::getLastUpdated()
{
	return this->timeLastUpdated;
}

void QuadcopterInfo::setLastUpdated(long int newLastUpdated)
{
	this->timeLastUpdated = newLastUpdated;
}

long int QuadcopterInfo::getShutdownStarted()
{
	return this->timeShutdownStarted;
}

void QuadcopterInfo::setShutdownStarted(long int newShutdownStarted)
{
	this->timeShutdownStarted = newShutdownStarted;
}

float QuadcopterInfo::checkAndFixRoll(float roll)
{
	if (roll < ROLL_MIN) {
		return ROLL_MIN;
	} else if (roll > ROLL_MAX)   {
		return ROLL_MAX;
	}
	return roll;
}

float QuadcopterInfo::checkAndFixPitch(float pitch)
{
	if (pitch < PITCH_MIN) {
		return PITCH_MIN;
	} else if (pitch > PITCH_MAX)   {
		return PITCH_MAX;
	}
	return pitch;
}

float QuadcopterInfo::checkAndFixYawrate(float yawrate)
{
	if (yawrate < YAWRATE_MIN) {
		return YAWRATE_MIN;
	} else if (yawrate > YAWRATE_MAX)   {
		return YAWRATE_MAX;
	}
	return yawrate;
}

double QuadcopterInfo::getX()
{
	return this->x;
}

double QuadcopterInfo::getY()
{
	return this->y;
}

double QuadcopterInfo::getRotation()
{
	return this->rotation;
}

void QuadcopterInfo::setX(double newX)
{
	this->x = newX;
}

void QuadcopterInfo::setY(double newY)
{
	this->y = newY;
}

void QuadcopterInfo::setRotation(double newRotation)
{
	this->rotation = newRotation;
}

bool QuadcopterInfo::isNegativeSign()
{
	return negativeSign;
}

void QuadcopterInfo::setNegativeSign(bool negativeSign)
{
	this->negativeSign = negativeSign;
}


#include "InterpolatorInfo.hpp"

InterpolatorInfo::InterpolatorInfo()
{
	this->state = UNSTARTED;
	/*
		this->lastUpdated[i] = 0;
		this->northeast[i][0] = INVALID;
		this->northeast[i][1] = INVALID;
		this->started[i] = 0;*/
	
}

/* Getter and Setter */
short InterpolatorInfo::getState()
{
    return this->state;
}

void InterpolatorInfo::setState( short newState )
{
    this->state = newState;
}


long int InterpolatorInfo::getStarted()
{
    return this->timeStarted;
}

void InterpolatorInfo::setStarted( long int newStarted )
{
    this->timeStarted = newStarted;
}

long int InterpolatorInfo::getLastUpdated()
{
    
    return this->timeLastUpdated;
}

void InterpolatorInfo::setLastUpdated( long int newLastUpdated )
{
    this->timeLastUpdated = newLastUpdated;
}

long int InterpolatorInfo::getShutdownStarted()
{
    return this->timeShutdownStarted;
}

void InterpolatorInfo::setShutdownStarted( long int newShutdownStarted )
{
	this->timeShutdownStarted = newShutdownStarted;
}

double InterpolatorInfo::getX()
{
    return this->x;
}

double InterpolatorInfo::getY()
{
    return this->y;
}

double InterpolatorInfo::getRotation()
{
    return this->rotation;
}

void InterpolatorInfo::setX( double newX )
{
    this->x = newX;
}

void InterpolatorInfo::setY( double newY )
{
    this->y = newY;
}

void InterpolatorInfo::setRotation( double newRotation )
{
    this->rotation = newRotation;
}

bool InterpolatorInfo::isNegativeSign()
{
	return negativeSign;
}

void InterpolatorInfo::setNegativeSign(bool negativeSign) {
	this->negativeSign = negativeSign;
}

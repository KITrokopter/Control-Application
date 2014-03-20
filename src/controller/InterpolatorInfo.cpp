#include "InterpolatorInfo.hpp"

InterpolatorInfo::InterpolatorInfo()
{
	state = UNSTARTED;
	/*
		this->lastUpdated[i] = 0;
		this->northeast[i][0] = INVALID;
		this->northeast[i][1] = INVALID;
		this->started[i] = 0;*/
	
}

/* Getter and Setter */
long int InterpolatorInfo::getStarted()
{
    return this->started;
}

void InterpolatorInfo::setStarted( long int newStarted )
{
    this->started = newStarted;
}

int InterpolatorInfo::getState()
{
    return this->state;
}

void InterpolatorInfo::setState( int newState )
{
    this->state = newState;
}

long int InterpolatorInfo::getLastUpdated()
{
    
    return this->lastUpdated;
}

void InterpolatorInfo::setLastUpdated( long int newLastUpdated )
{
    this->lastUpdated = newLastUpdated;
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


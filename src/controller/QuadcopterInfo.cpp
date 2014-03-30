/*
 * QuadcopterInfo.cpp
 *
 *  Created on: 30.03.2014
 *      Author: dwx
 */

#include "QuadcopterInfo.hpp"

QuadcopterInfo::QuadcopterInfo() {
	this->state = UNSTARTED;
}

/* Getter and Setter */
short QuadcopterInfo::getState()
{
	return this->state;
}

void QuadcopterInfo::setState( short newState )
{
	this->state = newState;
}


long int QuadcopterInfo::getStarted()
{
	return this->timeStarted;
}

void QuadcopterInfo::setStarted( long int newStarted )
{
	this->timeStarted = newStarted;
}

long int QuadcopterInfo::getLastUpdated()
{

	return this->timeLastUpdated;
}

void QuadcopterInfo::setLastUpdated( long int newLastUpdated )
{
	this->timeLastUpdated = newLastUpdated;
}

long int QuadcopterInfo::getShutdownStarted()
{
	return this->timeShutdownStarted;
}

void QuadcopterInfo::setShutdownStarted( long int newShutdownStarted )
{
	this->timeShutdownStarted = newShutdownStarted;
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

void QuadcopterInfo::setX( double newX )
{
	this->x = newX;
}

void QuadcopterInfo::setY( double newY )
{
	this->y = newY;
}

void QuadcopterInfo::setRotation( double newRotation )
{
	this->rotation = newRotation;
}

bool QuadcopterInfo::isNegativeSign()
{
	return negativeSign;
}

void QuadcopterInfo::setNegativeSign(bool negativeSign) {
	this->negativeSign = negativeSign;
}

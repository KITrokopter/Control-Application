#include "QuadcopterThrust.hpp"

QuadcopterThrust::QuadcopterThrust
{
	this->min = 28000;
	this->max = 40000;
	this->start = 28000;
	this->startMax = 42000;
	this->setThrustCalled = false;
}

bool QuadcopterThrust::checkAndSetBatteryValue( float battery )
{
	if( battery > BATTERY_MAX )
	{
		return false;
	} 
	else if( batter < BATTERY_LOW )
	{
		return false;
	}
	setThrust( battery );
	this->setThrustCalled = true;
	return true;
}

unsigned int QuadcopterThrust::checkAndFix( unsigned int thrust )
{
	if( thrust > this->smax )
	{
		return this->smax;
	}
	else if( thrust < this->smin )
	{
		return this->smin;
	}
	else return thrust;
}

void QuadcopterThrust::setThrust( float battery )
{
	if( battery > 4 )
	{
		return;
	}
	else if( battery > 3 )
	{
		this->min += ((unsigned int) ((battery-4) * QUADCOPTER_THRUST_RANGE));
		this->max += ((unsigned int) ((battery-4) * QUADCOPTER_THRUST_RANGE));
		this->start += ((unsigned int) ((battery-4) * QUADCOPTER_THRUST_RANGE));
		this->startMax += ((unsigned int) ((battery-4) * QUADCOPTER_THRUST_RANGE));
	}
	else 
	{
		this->min += QUADCOPTER_THRUST_RANGE;
		this->max += QUADCOPTER_THRUST_RANGE;
		this->start += QUADCOPTER_THRUST_RANGE;
		this->startMax += QUADCOPTER_THRUST_RANGE;
	}
}

bool QuadcopterThrust::initDone()
{
	return this->setThrustCalled;
}

void QuadcopterThrust::setMin( unsigned int min )
{
	this->min = min;
}

unsigned int QuadcopterThrust::getMin()
{
	return min;
}

void QuadcopterThrust::setMax( unsigned int max )
{
	this->max = max;
}

unsigned int QuadcopterThrust::getMax()
{
	return max;
}

void QuadcopterThrust::setStartMax( unsigned int startMax )
{
	this->startMax = startMax;
}

unsigned int QuadcopterThrust::getStartMax()
{
	return startMax;
}

void QuadcopterThrust::setStart( unsigned int start )
{
	this->start = start;
}

unsigned int QuadcopterThrust::getStart()
{
	return start;
}

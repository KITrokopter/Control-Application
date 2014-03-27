#include "QuadcopterThrust.hpp"

QuadcopterThrust::QuadcopterThrust()
{
	this->min = 22000;
	this->max = 44000;
	this->start = 30000;
	this->startMax = 52000;
	this->setThrustCalled = false;
}

bool QuadcopterThrust::checkAndSetBatteryValue( float battery )
{
	if( battery > BATTERY_MAX )
	{
		ROS_ERROR("checkAndSet +");
		return false;
	} 
	else if( battery < BATTERY_LOW )
	{
		ROS_ERROR("");
		ROS_ERROR("checkAndSet -");
		return false;
	}
	ROS_ERROR("checkAndSet working");
	setThrust( battery );
	this->setThrustCalled = true;
	return true;
}

unsigned int QuadcopterThrust::checkAndFix( unsigned int thrust )
{
	if( thrust > this->max )
	{
		return this->max;
	}
	else if( thrust < this->min )
	{
		return this->min;
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
		this->min += ((unsigned int) ((4-battery) * QUADCOPTER_THRUST_RANGE));
		this->max += ((unsigned int) ((4-battery) * QUADCOPTER_THRUST_RANGE));
		this->start += ((unsigned int) ((4-battery) * QUADCOPTER_THRUST_RANGE));
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

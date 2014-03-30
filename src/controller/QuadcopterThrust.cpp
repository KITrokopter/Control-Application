#include "QuadcopterThrust.hpp"

QuadcopterThrust::QuadcopterThrust()
{
	init();
	this->setThrustCalled = false;
}

void QuadcopterThrust::init()
{
	this->min = THRUST_GLOBAL_MIN;
	this->max = THRUST_GLOBAL_MAX;
	this->start = 30000;
	this->startMax = 50000;
	this->decline = 25000;
}

bool QuadcopterThrust::checkAndSetBatteryValue( float battery )
{
	if( battery > BATTERY_MAX )
	{
		ROS_ERROR("checkAndSet +");
		return false;
	} 
	else if( battery < BATTERY_MIN )
	{
		//ROS_ERROR("checkAndSet -");
		return false;
	}
	ROS_DEBUG("checkAndSet working");
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

unsigned int QuadcopterThrust::checkAndFix(double thrust)
{
	if( thrust < 0 )
	{
		return 0;
	}
	else if( thrust > this->max )
	{
		return this->max;
	}
	else
	{
		unsigned int newThrust = thrust;
		return checkAndFix( newThrust );
	}
}

unsigned int QuadcopterThrust::checkAndFix(double thrust)
{
	if( thrust < 0 )
	{
		return 0;
	}
	else if( thrust > this->max )
	{
		return this->max;
	}
	else
	{
		unsigned int newThrust = thrust;
		return checkAndFix( newThrust );
	}
}

void QuadcopterThrust::setWithoutBatteryValue()
{
	init();
	this->setThrustCalled = true;
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
		this->startMax += ((unsigned int) ((4-battery) * QUADCOPTER_THRUST_RANGE));
	}
	else 
	{
		this->min += QUADCOPTER_THRUST_RANGE;
		this->max += QUADCOPTER_THRUST_RANGE;
		this->start += QUADCOPTER_THRUST_RANGE;
		this->startMax += QUADCOPTER_THRUST_RANGE;
	}
	ROS_INFO("min %i, max %i, start %i, startMax %i", min, max, start, startMax);
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

void QuadcopterThrust::setDecline( unsigned int decline )
{
	this->decline = decline;
}

unsigned int QuadcopterThrust::getDecline()
{
	return decline;
}

#include "QuadcopterThrust.hpp"

/**
 * Constructor with calling init() at start.
 */
QuadcopterThrust::QuadcopterThrust()
{
	init();
	this->setThrustCalled = false;
}

/**
 * Initializing with random values, that seemed to be working for a fully charged quadcopter.
 * setThrustCalled is not changed here.
 */
void QuadcopterThrust::init()
{
	this->min = THRUST_GLOBAL_MIN;
	this->max = THRUST_GLOBAL_MAX;
	this->start = 30000;
	this->startMax = 50000;
	this->decline = 25000;
	this->offset = 32000;
}

/**
 * Check if received value seems to be realistic. Set if is.
 * @param battery Value to check
 * @return True if setThrust(_) was called.
 */
bool QuadcopterThrust::checkAndSetBatteryValue(float battery)
{
	if (battery > BATTERY_MAX) {
		ROS_ERROR("checkAndSet +");
		return false;
	} else if (battery < BATTERY_MIN)   {
		ROS_ERROR("checkAndSet -");
		return false;
	}
	setThrust(battery);
	this->setThrustCalled = true;
	return true;
}

unsigned int QuadcopterThrust::checkAndFix(unsigned int thrust)
{
	if (thrust > this->max) {
		return this->max;
	} else if (thrust < this->min)   {
		return this->min;
	} else   {return thrust;
	}
}

unsigned int QuadcopterThrust::checkAndFix(double thrust)
{
	if (thrust > this->max) {
		return this->max;
	} else if (thrust < this->min)   {
		return this->min;
	} else   {
		unsigned int newThrust = thrust;
		return checkAndFix(newThrust);
	}
}

void QuadcopterThrust::setWithoutBatteryValue()
{
	if (!setThrustCalled) {
		init();
		this->setThrustCalled = true;
	}
}

/**
 * It currently can be set more than once.
 * @param battery Set thrustvalues battery-dependent.
 */
void QuadcopterThrust::setThrust(float battery)
{
	if (battery > 4) {
		return;
	} else if (battery > 3)   {
		this->min += ((unsigned int) ((4 - battery) * QUADCOPTER_THRUST_RANGE));
		this->max += ((unsigned int) ((4 - battery) * QUADCOPTER_THRUST_RANGE));
		this->start += ((unsigned int) ((4 - battery) * QUADCOPTER_THRUST_RANGE));
		this->startMax += ((unsigned int) ((4 - battery) * QUADCOPTER_THRUST_RANGE));
		// ROS_INFO("min %i, max %i, start %i, startMax %i", min, max, start,
		// startMax);
		this->setThrustCalled = true;
	} else   {
		this->min += QUADCOPTER_THRUST_RANGE;
		this->max += QUADCOPTER_THRUST_RANGE;
		this->start += QUADCOPTER_THRUST_RANGE;
		this->startMax += QUADCOPTER_THRUST_RANGE;
		// ROS_INFO("min %i, max %i, start %i, startMax %i", min, max, start,
		// startMax);
		this->setThrustCalled = true;
	}
}

bool QuadcopterThrust::initDone()
{
	return this->setThrustCalled;
}

void QuadcopterThrust::setMin(unsigned int min)
{
	this->min = min;
}

unsigned int QuadcopterThrust::getMin()
{
	return min;
}

void QuadcopterThrust::setMax(unsigned int max)
{
	this->max = max;
}

unsigned int QuadcopterThrust::getMax()
{
	return max;
}

void QuadcopterThrust::setStartMax(unsigned int startMax)
{
	this->startMax = startMax;
}

unsigned int QuadcopterThrust::getStartMax()
{
	return startMax;
}

void QuadcopterThrust::setStart(unsigned int start)
{
	this->start = start;
}

unsigned int QuadcopterThrust::getStart()
{
	return start;
}

void QuadcopterThrust::setDecline(unsigned int decline)
{
	this->decline = decline;
}

unsigned int QuadcopterThrust::getDecline()
{
	return this->decline;
}

void QuadcopterThrust::setOffset(float battery)
{
	if (battery > 4.2) {
		this->offset = THRUST_OFFSET_LOW;
	} else if (battery > 2.8)   {
		this->offset = THRUST_OFFSET_LOW + ((unsigned int) ((4.2 - battery) * QUADCOPTER_THRUST_RANGE));
	} else   {
		this->offset = THRUST_OFFSET_LOW + QUADCOPTER_THRUST_RANGE;
	}
}

unsigned int QuadcopterThrust::getOffset()
{
	return this->offset;
}


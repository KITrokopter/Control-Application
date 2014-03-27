#include "QuadcopterThrust.hpp"
#include "Controller.hpp"

QuadcopterThrust::QuadcopterThrust
{
	min = 28000;
	max = 40000;
	start = 28000;
	startMax = 42000;
}

void setThrust( double battery )
{

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
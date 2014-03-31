#include "QuadcopterControl.hpp"

QuadcopterControl::QuadcopterControl()
{
	this->info = QuadcopterInfo();
	this->thrust = QuadcopterThrust();
}
QuadcopterInfo QuadcopterControl::getInfo()
{
	return this->info;
}

void QuadcopterControl::setInfo( QuadcopterInfo newInfo )
{
	this->info = newInfo;
}

QuadcopterThrust QuadcopterControl::getThrust()
{
	return this->thrust;
}

void QuadcopterControl::setThrust( QuadcopterThrust newThrust )
{
	this->thrust = newThrust;
}
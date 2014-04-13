#include "QuadcopterControl.hpp"

QuadcopterControl::QuadcopterControl()
{
	// this->info = QuadcopterInfo();
	// this->quadcopterThrust = QuadcopterThrust();
}

QuadcopterInfo QuadcopterControl::getInfo()
{
	return this->info;
}

void QuadcopterControl::setInfo(QuadcopterInfo newInfo)
{
	this->info = newInfo;
}

QuadcopterThrust QuadcopterControl::getQuadcopterThrust()
{
	return this->quadcopterThrust;
}

void QuadcopterControl::setQuadcopterThrust(QuadcopterThrust newQuadcopterThrust)
{
	this->quadcopterThrust = newQuadcopterThrust;
}


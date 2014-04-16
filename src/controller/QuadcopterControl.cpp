#include "QuadcopterControl.hpp"

/**
 * Empty constructor.
 */
QuadcopterControl::QuadcopterControl()
{
}

/**
 * Standard getter.
 * @return QuadcopterInfo
 */
QuadcopterInfo QuadcopterControl::getInfo()
{
	return this->info;
}

/**
 * Standard setter.
 * @param newInfo Setting new QuadcopterInfo
 */
void QuadcopterControl::setInfo(QuadcopterInfo newInfo)
{
	this->info = newInfo;
}

/**
 * Standard getter.
 * @return QuadcopterControl
 */
QuadcopterThrust QuadcopterControl::getQuadcopterThrust()
{
	return this->quadcopterThrust;
}

/**
 * Standard setter.
 * @param newQuadcopterThrust Setting new QuadcopterThrust
 */
void QuadcopterControl::setQuadcopterThrust(QuadcopterThrust newQuadcopterThrust)
{
	this->quadcopterThrust = newQuadcopterThrust;
}


#include "PControl.hpp"

/**
 * Can be used as normal P-Controller with offset.
 * @param pFactor proportional factor
 * @param offset Offset added to calculated variable
 */
PControl::PControl(double pFactor, double offset)
{
	this->pAmplification = pFactor;
	this->pAmplificationPos = pFactor;
	this->pAmplificationNeg = pFactor;
	this->offset = offset;
}

/**
 * A P-Controller with two p-factors.
 * @param pFactorPos Factor used if error is >= 0
 * @param pFactorNeg Factor used if error is < 0
 * @param offset Offset added to calculated variable
 */
PControl::PControl(double pFactorPos, double pFactorNeg, double offset)
{
	this->pAmplification = pFactorPos;
	this->pAmplificationPos = pFactorPos;
	this->pAmplificationNeg = pFactorNeg;
	this->offset = offset;
}

double PControl::getManipulatedVariable(double errorSignal)
{
	setPAmplification(errorSignal);
	double y = this->pAmplification * errorSignal + this->offset;
	return y;
}

void PControl::setOffset(double offset)
{
	this->offset = offset;
}

/**
 * Set p-factor to pFactorPos if error is >= 0.
 * @param errorSignal sign of parameter is deciding
 */
void PControl::setPAmplification(double errorSignal)
{
	if (errorSignal >= 0.0) {
		this->pAmplification = this->pAmplificationPos;
	} else   {
		this->pAmplification = this->pAmplificationNeg;
	}
}


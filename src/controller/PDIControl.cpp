#include "PDIControl.hpp"

/**
 * A PD-Controller with i-factor set to zero.
 * @param pFactor Proportional factor in Controller
 * @param dFactor Differential factor in Controller
 * @param offset Offset added to calculated variable
 */
PDIControl::PDIControl(double pFactor, double dFactor, double offset)
{
	this->pAmplification = pFactor;
	this->pAmplificationPos = pFactor;
	this->pAmplificationNeg = pFactor;
	this->dAmplification = dFactor;
	this->iAmplification = 0.0;
	this->offset = offset;
	this->distanceOld = 0.0;
	this->integrator = 0.0;
}

/**
 * A PD-Controller with i-factor set to zero and two p-factors.
 * @param pFactorPos Factor used if error is >= 0
 * @param pFactorNeg Factor used if error is < 0
 * @param dFactor Differential factor in Controller
 * @param offset Offset added to calculated variable
 */
PDIControl::PDIControl(double pFactorPos, double pFactorNeg, double dFactor, double offset)
{
	this->pAmplification = pFactorPos;
	this->pAmplificationPos = pFactorPos;
	this->pAmplificationNeg = pFactorNeg;
	this->dAmplification = dFactor;
	this->iAmplification = 0.0;
	this->offset = offset;
	this->distanceOld = 0.0;
	this->integrator = 0.0;
}

/**
 * A PDI-Controller with two p-factors.
 * @param pFactorPos Factor used if error is >= 0
 * @param pFactorNeg Factor used if error is < 0
 * @param dFactor Differential factor in Controller
 * @param iFactor Integrating factor in Controller
 * @param offset Offset added to calculated variable
 */
PDIControl::PDIControl(double pFactorPos, double pFactorNeg, double dFactor, double iFactor, double offset)
{
	this->pAmplification = pFactorPos;
	this->pAmplificationPos = pFactorPos;
	this->pAmplificationNeg = pFactorNeg;
	this->dAmplification = dFactor;
	this->iAmplification = iFactor;
	this->offset = offset;
	this->distanceOld = 0.0;
	this->integrator = 0.0;
}

double PDIControl::getManipulatedVariable(double errorSignal)
{
	setPAmplification(errorSignal);
	double distanceDiff = errorSignal - distanceOld;
	double p = this->pAmplification * errorSignal;
	double d = this->pAmplification * (this->dAmplification * distanceDiff);
	this->integrator += errorSignal;
	double i = this->integrator * this->iAmplification;
	double y = p + d + i + offset;
	this->distanceOld = errorSignal;
	ROS_DEBUG("error %f, distDiff %f, offset %f", errorSignal, distanceDiff, offset);
	ROS_DEBUG("p %f, d %f, i %f, y %f", p, d, i, y);
	return y;
}

void PDIControl::setOffset(double offset)
{
	this->offset = offset;
}

/**
 * Set p-factor to pFactorPos if error is >= 0.
 * @param errorSignal Sign of errorSignal is used.
 */
void PDIControl::setPAmplification(double errorSignal)
{
	if (errorSignal >= 0.0) {
		this->pAmplification = this->pAmplificationPos;
	} else   {
		ROS_DEBUG("Error signal negative. Above target");
		this->pAmplification = this->pAmplificationNeg;
	}
}


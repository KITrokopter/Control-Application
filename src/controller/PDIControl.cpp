#include "PDIControl.hpp"

PDIControl::PDIControl( double pFactor, double dFactor, double offset )
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

PDIControl::PDIControl( double pFactorPos, double pFactorNeg, double dFactor, double offset )
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

PDIControl::PDIControl( double pFactorPos, double pFactorNeg, double dFactor, double iFactor, double offset )
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
	setPAmplification( errorSignal );
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

void PDIControl::setOffset( double offset )
{
	this->offset = offset;
}

void PDIControl::setPAmplification( double errorSignal )
{
	if( errorSignal >= 0.0 )
	{
		this->pAmplification = this->pAmplificationPos;
	}
	else
	{
		this->pAmplification = this->pAmplificationNeg;
	}
}

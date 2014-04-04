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
	double y = this->pAmplification * (errorSignal + (this->dAmplification * distanceDiff));	
	this->integrator += errorSignal;
	y += this->integrator * this->iAmplification;
	y += this->offset;	
	this->distanceOld = errorSignal;	
	return y;
}

double PDIControl::getAmplification()
{
	return this->pAmplification;
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

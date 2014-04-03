/*
 * PControl.cpp
 *
 *  Created on: 30.03.2014
 *      Author: dwx
 */

#include "PControl.hpp"

PControl::PControl( double pFactor, double offset )
{
	this->pAmplification = pFactor;
	this->pAmplificationPos = pFactor;
	this->pAmplificationNeg = pFactor;
	this->dAmplification = 0.0;
	this->offset = offset;
	this->distanceOld = 0.0;
}

PControl::PControl( double pFactor, double dFactor, double offset )
{
	this->pAmplification = pFactor;
	this->pAmplificationPos = pFactor;
	this->pAmplificationNeg = pFactor;
	this->dAmplification = dFactor;
	this->offset = offset;
	this->distanceOld = 0.0;
}

PControl::PControl( double pFactorPos, double pFactorNeg, double dFactor, double offset )
{
	this->pAmplification = pFactorPos;
	this->pAmplificationPos = pFactorPos;
	this->pAmplificationNeg = pFactorNeg;
	this->dAmplification = dFactor;
	this->offset = offset;
	this->distanceOld = 0.0;
}

double PControl::getManipulatedVariable(double errorSignal)
{
	setPAmplification( errorSignal );
	double distanceDiff = errorSignal - distanceOld;
	double y = this->pAmplification * (errorSignal + (dAmplification * distanceDiff));
	y += this->offset;
	this->distanceOld = errorSignal;
	return y;
	
}

double PControl::getAmplification()
{
	this->pAmplification;
}

void PControl::setPAmplification( double errorSignal )
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

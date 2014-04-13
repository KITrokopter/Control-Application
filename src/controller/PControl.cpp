#include "PControl.hpp"

PControl::PControl(double pFactor, double offset)
{
	this->pAmplification = pFactor;
	this->pAmplificationPos = pFactor;
	this->pAmplificationNeg = pFactor;
	this->offset = offset;
}

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

void PControl::setPAmplification(double errorSignal)
{
	if (errorSignal >= 0.0) {
		this->pAmplification = this->pAmplificationPos;
	} else   {
		this->pAmplification = this->pAmplificationNeg;
	}
}


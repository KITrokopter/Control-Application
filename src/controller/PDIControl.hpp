#ifndef PDICONTROL_HPP_
#define PDICONTROL_HPP_

#include "Control.hpp"


class PDIControl : public Control {

public:
	PDIControl( double pFactor, double dFactor, double offset );
	PDIControl( double pFactorPos, double pFactorNeg, double dFactor, double offset );
	PDIControl( double pFactorPos, double pFactorNeg, double dFactor, double iFactor, double offset );

	double getAmplification();
	double getManipulatedVariable( double errorSignal );
	void setOffset( double offset );

protected:
	void setPAmplification( double errorSignal );
	
private:
	
	double pAmplification;
	double pAmplificationPos;
	double pAmplificationNeg;
	double dAmplification;	
	double iAmplification;
	double offset;
	double distanceOld;
	double integrator;

};

#endif /* PDICONTROL_HPP_ */

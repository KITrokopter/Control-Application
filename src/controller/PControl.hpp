#ifndef PCONTROL_HPP_
#define PCONTROL_HPP_

#include "Control.hpp"


class PControl : public Control {

public:
    	PControl( double pFactor, double offset );
    	PControl( double pFactor, double dFactor, double offset );
    	PControl( double pFactorPos, double pFactorNeg, double dFactor, double offset );

	double getManipulatedVariable( double errorSignal );

protected:
	void setPAmplification( double errorSignal );
	
private:
	
	double pAmplification;
	double pAmplificationPos;
	double pAmplificationNeg;
	double dAmplification;	
	double offset;
	double distanceOld;
};

#endif /* PCONTROL_HPP_ */

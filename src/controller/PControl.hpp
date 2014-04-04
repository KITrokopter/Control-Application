#ifndef PCONTROL_HPP_
#define PCONTROL_HPP_

#include "Control.hpp"


class PControl : public Control {

public:
	PControl( double pFactor, double offset );

	double getAmplification();
	double getManipulatedVariable( double errorSignal );

protected:
	void setPAmplification( double errorSignal );
	
private:
	
	double pAmplification;
	double pAmplificationPos;
	double pAmplificationNeg;
	double offset;
};

#endif /* PCONTROL_HPP_ */

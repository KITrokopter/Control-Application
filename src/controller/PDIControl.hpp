#ifndef PDICONTROL_HPP_
#define PDICONTROL_HPP_

#include "Control.hpp"
#include "ros/ros.h"


class PDIControl : public Control {

public:

	/**
	 * A PD-Controller with i-factor set to zero.
	 * @param pFactor Proportional factor in Controller
	 * @param dFactor Differential factor in Controller
	 * @param offset Offset added to calculated variable
	 */
	PDIControl( double pFactor, double dFactor, double offset );

	/**
	 * A PD-Controller with i-factor set to zero and two p-factors.
	 * @param pFactorPos Factor used if error is >= 0
	 * @param pFactorNeg Factor used if error is < 0
	 * @param dFactor Differential factor in Controller
	 * @param offset Offset added to calculated variable
	 */
	PDIControl( double pFactorPos, double pFactorNeg, double dFactor, double offset );

	/**
	 * A PDI-Controller with two p-factors.
	 * @param pFactorPos Factor used if error is >= 0
	 * @param pFactorNeg Factor used if error is < 0
	 * @param dFactor Differential factor in Controller
	 * @param iFactor Integrating factor in Controller
	 * @param offset Offset added to calculated variable
	 */
	PDIControl( double pFactorPos, double pFactorNeg, double dFactor, double iFactor, double offset );

	double getManipulatedVariable( double errorSignal );
	void setOffset( double offset );

protected:

	/**
	 * Set p-factor to pFactorPos if error is >= 0.
	 * @param errorSignal Sign of errorSignal is used.
	 */
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

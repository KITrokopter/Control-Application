#ifndef PCONTROL_HPP_
#define PCONTROL_HPP_

#include "Control.hpp"


class PControl : public Control {
public:

	/**
	 * Can be used as normal P-Controller with offset.
	 * @param pFactor proportional factor
	 * @param offset Offset added to calculated variable
	 */
	PControl(double pFactor, double offset);

	/**
	 * A P-Controller with two p-factors.
	 * @param pFactorPos Factor used if error is >= 0
	 * @param pFactorNeg Factor used if error is < 0
	 * @param offset Offset added to calculated variable
	 */
	PControl(double pFactorPos, double pFactorNeg, double offset);

	double getManipulatedVariable(double errorSignal);
	void setOffset(double offset);

protected:

	/**
	 * Set p-factor to pFactorPos if error is >= 0.
	 * @param errorSignal Sign of errorSignal is used.
	 */
	void setPAmplification(double errorSignal);

private:

	double pAmplification;
	double pAmplificationPos;
	double pAmplificationNeg;
	double offset;
};

#endif /* PCONTROL_HPP_ */

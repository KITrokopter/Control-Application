#ifndef PCONTROL_HPP_
#define PCONTROL_HPP_

#include "Control.hpp"

/**
 * A P-Controller with the possibility to set different factors for a given positive or negative error.
 * Useful for thrust-regulation in some cases.
 *
 * @author Dominik Kiefer
 */
class PControl : public Control {
public:
	PControl(double pFactor, double offset);
	PControl(double pFactorPos, double pFactorNeg, double offset);
	double getManipulatedVariable(double errorSignal);
	void setOffset(double offset);

protected:
	void setPAmplification(double errorSignal);

private:
	double pAmplification;
	double pAmplificationPos;
	double pAmplificationNeg;
	double offset;
};

#endif /* PCONTROL_HPP_ */

#ifndef CONTROL_HPP_
#define CONTROL_HPP_

/**
 * A virtual parent class for standard controller:
 * P-, PD-, PDI-Controller.
 */
class Control {
public:

	/**
	 * Central method of the Controller.
	 * @param errorSignal Input
	 * @return manipulated or controlled variable
	 */
	virtual double getManipulatedVariable(double errorSignal) = 0;

	/**
	 * Used if offset changes during time, e.g. battery-dependent.
	 * @param offset Used offset
	 */
	virtual void setOffset(double offset) = 0;
};

#endif /* CONTROL_HPP_ */

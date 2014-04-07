#ifndef CONTROL_HPP_
#define CONTROL_HPP_

class Control {
public:

	/**
	 * Central method of the Controller.
	 * @param errorSignal Input
	 * @return manipulated or controlled variable
	 */
	virtual double getManipulatedVariable( double errorSignal ) = 0;

	/**
	 *
	 * @return
	 */
	virtual double getAmplification() = 0;

	/**
	 *
	 * @param offset
	 */
	virtual void setOffset( double offset ) = 0;

};

#endif /* CONTROL_HPP_ */

/*
 * Control.hpp
 *
 *  Created on: 30.03.2014
 *      Author: dwx
 */

#ifndef CONTROL_HPP_
#define CONTROL_HPP_

class Control {
public:

	/**
	 *
	 * @param errorSignal
	 * @return
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

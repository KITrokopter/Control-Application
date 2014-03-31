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
	Control();

	virtual double getManipulatedVariable( double errorSignal );

};

#endif /* CONTROL_HPP_ */

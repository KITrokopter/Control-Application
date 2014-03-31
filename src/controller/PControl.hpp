/*
 * PControl.hpp
 *
 *  Created on: 30.03.2014
 *      Author: dwx
 */

#ifndef PCONTROL_HPP_
#define PCONTROL_HPP_

#include "Control.hpp"


class PControl : public Control {

public:
	PControl( double amplificationFactor );

	double getManipulatedVariable( double errorSignal );

private:
	double amplification;
};

#endif /* PCONTROL_HPP_ */

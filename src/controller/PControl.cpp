/*
 * PControl.cpp
 *
 *  Created on: 30.03.2014
 *      Author: dwx
 */

#include "PControl.hpp"

PControl::PControl()
{
	this->amplification = AMPLIFICATION_FACTOR;

}

double PControl::getManipulatedVariable(double errorSignal)
{
	return (this->amplification * errorSignal);
}

/*
 * PControl.cpp
 *
 *  Created on: 30.03.2014
 *      Author: dwx
 */

#include "PControl.hpp"

PControl::PControl( double amplificationFactor, double offset )
{
	this->amplification = amplificationFactor;
	this->offset = offset;

}

double PControl::getManipulatedVariable(double errorSignal)
{
	return (this->amplification * errorSignal + offset);
}

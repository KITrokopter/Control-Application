#include "CalculatorMoveUp.hpp"
#include "Controller.hpp"

CalculatorMoveUp::CalculatorMoveUp
{
	this->chargeToThrust.insert( std::pair<double, int>(0, 60000));
	this->chargeToThrust.insert( std::pair<double, int>(3, 10001));	
	this->chargeToThrust.insert( std::pair<double, int>(1.5, 40001));
}

int CalculatorMoveUp::thrustStartLinear( double charge, int amount )
{

}
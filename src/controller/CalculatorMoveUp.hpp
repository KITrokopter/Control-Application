#ifndef CALCULATOR_MOVE_UP_HPP
#define CALCULATOR_MOVE_UP_HPP

#include <sys/time.h>
#include <map>

class CalculatorMoveUp
{	
	public:
		CalculatorMoveUp();

	private:
		std::map<double, int> chargeToThrust;

};

#endif // CALCULATOR_MOVE_UP_HPP

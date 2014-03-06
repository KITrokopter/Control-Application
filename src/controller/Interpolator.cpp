#include "Interpolator.hpp"

Interpolator::Interpolator()
{
	this->speedOfChange = 1;
}

Interpolator::MovementQuadruple calculateNextMQ(<std::list<MovementQuadruple> sentQuadruples, std::list<Position6DOF> positions)
{

	if( sentQuadruples.size() < 5 || positions.size() < 5 )
	{
		return MovementQuadruple(-1, -1, -1, -,1);
	}

	int counter = 0;
	for(std::list<Position6DOF>::positions it = positions.begin(); it != positions.end(); ++it)
	{
		
		counter++;
	}
	MovementQuadruple newMovement = MovementQuadruple(0, 0, 0, 0);

	return newMovement;
}
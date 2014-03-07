#include "Interpolator.hpp"

Interpolator::Interpolator()
{
	this->speedOfChange = 1;
	for( int i = 0; i < MAX_NUMBER_OF_QUADCOPTER_HIGH; i++ )
	{
		this->lastUpdated[i] = 0;
	}
}

Interpolator::MovementQuadruple calculateNextMQ(<std::list<MovementQuadruple> sentQuadruples, std::list<Position6DOF> positions, int id)
{

	if( sentQuadruples.size() == 0 ) // FIXME
	{
		return MovementQuadruple(THRUST_START, 0, 0, 0);
	}
	
	MovementQuadruple newMovement = sentQuadruples.last();  // FIXME
	/* get current Time */
	
	if( id >= MAX_NUMBER_OF_QUADCOPTER_HIGH ) 
	{		
		ROS_INFO("Got wrong id in calculateNextMQ.");
		return newMovement;
	} else if( sentQuadruples.size() < 3 || positions.size() < 3 )
	{
		ROS_INFO("Not enough data in calculateNextMQ.");
		return newMovement;
	} else if( this->lastUpdated[id] - currentTime < MIN_TIME_TO_WAIT) // FIXME
	{
		ROS_INFO("Take old value, changes need to be visible for calculateNextMQ.");
		return newMovement;
	}

	ROS_INFO("Enough data in calculateNextMQ, start calculation.");
	int counter = 0;
	for(std::list<Position6DOF>::positions it = positions.begin(); it != positions.end(); ++it)
	{
		
		counter++;
	}

	return newMovement;
}
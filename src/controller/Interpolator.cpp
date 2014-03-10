#include "Interpolator.hpp"
#include "Controller.hpp"

Interpolator::Interpolator()
{
	this->stepSizeOfChange = 1;
	for( int i = 0; i < MAX_NUMBER_OF_QUADCOPTER_HIGH; i++ )
	{
		this->lastUpdated[i] = 0;
	}
}

MovementQuadruple Interpolator::calculateNextMQ(std::list<MovementQuadruple> sentQuadruples, std::list<Position6DOF> positions, Position6DOF target, int id)
{

	if( sentQuadruples.size() == 0 )
	{
		return MovementQuadruple(THRUST_START, 0, 0, 0);
	}
	
	MovementQuadruple newMovement = sentQuadruples.back();
	long int currentTime = getNanoTime();
	
	if( id >= MAX_NUMBER_OF_QUADCOPTER_HIGH ) 
	{		
		ROS_INFO("Got wrong id in calculateNextMQ.");
		return newMovement;
	} else if( sentQuadruples.size() < 3 || positions.size() < 3 )
	{
		ROS_INFO("Not enough data in calculateNextMQ.");
		return newMovement;
	} else if( this->lastUpdated[id] - currentTime < MIN_TIME_TO_WAIT)
	{
		ROS_INFO("Take old value, changes need to be visible for calculateNextMQ.");
		return newMovement;
	}

	ROS_INFO("Enough data in calculateNextMQ, start calculation.");
	bool oscillate = false;
	double deltaTarget[positions.size()];
	double deltaAbsPosition[positions.size()-1];	// equals speed	/* error-prone FIXME */
	double deltaSpeed[positions.size()-2];	// equals acceleration	/* error-prone FIXME */
	int counter = 0;
	Position6DOF positionA, positionB;	// positionA is older than positionB
//	long int timeA, timeB;	// timeA is older than timeB
	for(std::list<Position6DOF>::iterator it = positions.begin(); it != positions.end(); ++it)
	{
		positionA.setOrientation( (*it).getOrientation() );
		positionA.setPosition( (*it).getPosition() );
		positionA.setTimestamp( (*it).getTimestamp() );
		deltaTarget[counter] = positionA.getAbsoluteDistance( target );
		if( counter > 0 )
		{
			deltaAbsPosition[counter-1] = positionB.getAbsoluteDistance( positionA );
			if( counter > 1 )
			{
				double speedDelta = deltaAbsPosition[counter-1] - deltaAbsPosition[counter-2];	
				if( speedDelta < 0 )
				{
					oscillate = true;
					speedDelta = -speedDelta;
				}
				double timeDelta = positionB.getTimestamp() - positionA.getTimestamp();
				deltaSpeed[counter-2] = speedDelta / timeDelta;
			}
		} 
		positionB = positionA;
		counter++;
	}

	return newMovement;
}

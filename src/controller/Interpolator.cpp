#include "Interpolator.hpp"
#include "Controller.hpp"

int calculateThrust(int thrust, double zDistanceFirst, double zDistanceLatest, double absDistanceFirstLatest, long int timediff);

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
	
	if( sentQuadruples.size() < 3 || positions.size() < 3 )
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
	int size = positions.size();
	double deltaTarget[size];
	double deltaAbsPosition[size-1];	// equals speed	/* error-prone FIXME */
	double deltaSpeed[size-2];	// equals acceleration	/* error-prone FIXME */
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
		if( positionA.getTimestamp() != positions.back().getTimestamp() )
		{
			positionB = positionA;
		}
		counter++;
	}

	/* Calculate thrust value */	
	if( counter > 0 )	// Enough data to calculate new thrust value
	{
		/*positionA.setPosition( positions.back().getPosition() );
		positionA.getTimestamp( positions.back().getTimestamp() );*/
		double zDistanceA = positionA.getDistanceZ( target );
		double zDistanceB = positionB.getDistanceZ( target );
		double timediffAB = positionB.getTimestamp() - positionA.getTimestamp();
		double absDistanceAB = positionA.getAbsoluteDistance( positionB );
		newMovement.setThrust( calculateThrust(newMovement.getThrust(), zDistanceA, zDistanceB, absDistanceAB, timediffAB) );
	}

	/* Calculate rest */
	if( counter > 1 )	// Enough data to calculate new roll/pitch/(yaw) values
	{
		
	}
	
	return newMovement;
}

int calculateThrust( int thrust, double zDistanceFirst, double zDistanceLatest, double absDistanceFirstLatest, long int timediff )
{
	int newThrust = thrust;
	double timediffNormalized = (double) (timediff / 1000000000);	// should be in seconds
	double distanceFactor = 0.5; // higher if further from target, between [0, 1]	//TODO
	double threshold = 0;	// higher if timediff is higher and 	//TODO

	/* Height-difference calculated as z-speed in mm/s. Positive if inclining. */
	double zSpeed = (zDistanceFirst-zDistanceLatest) * timediffNormalized;	// in mm/s
	
	/* 
	 * Do not change thrust if
	 * 	is inclining and "close" to target
	 * 	is declining and "close" to target
	 * Increase thrust if
	 * 	inclining too slow
	 * 	declining too fast
	 * 	positive distance to target is increasing
	 * Decrease thrust if
	 * 	inclining too fast
	 * 	declining too slow
	 * 	negative distance to target is increasing
	 * 	(speed is too high)
	 * 
	 * All that iff received position-values seem realistic 	TODO
	 */
	
	if( abs(zDistanceLatest) < DISTANCE_CLOSE_TO_TARGET ) 
	{
		return newThrust;		
	} else
	{
		if((zSpeed>0 && zSpeed<SPEED_MIN_INCLINING) || (zSpeed<SPEED_MAX_DECLINING) || (zDistanceLatest>0 && zDistanceLatest>zDistanceFirst)) 
		{  
			newThrust += THRUST_STEP;	
		}
		if((zSpeed>SPEED_MAX_INCLINING) || (zSpeed<0 && zSpeed>SPEED_MIN_DECLINING) || (zDistanceLatest<0 && zDistanceLatest<zDistanceFirst)) 
		{  
			newThrust -= THRUST_STEP;	
		}
		return newThrust;	
	}
}

bool reachingTarget( double first, double last, double speed, long int timediff )
{
	double timediffNormalized = (double) (timediff / 1000000000);	// should be in seconds
	double distanceFactor = 0.5; // higher if further from target, between [0, 1]
	double factor = REACHING_TARGET_DIFF * timediffNormalized * distanceFactor;
	if( (last<first) && ((first-last) > factor*speed) )
	{
		return true;
	}
	return false;
}

#include "Interpolator.hpp"
#include "Controller.hpp"

int calculateThrustDiff( double zDistanceFirst, double zDistanceLatest, double absDistanceFirstLatest, double timediffNormalized );
float calculatePlaneDiff( double aDistanceFirst, double aDistanceLatest, double absDistanceFirstLatest, double timediffNormalized, double aSentLatest );

Interpolator::Interpolator()
{
	this->stepSizeOfChange = 1;
	for( int i = 0; i < MAX_NUMBER_OF_QUADCOPTER_HIGH; i++ )
	{
		this->lastUpdated[i] = 0;
		this->northeast[i][0] = INVALID;
		this->northeast[i][1] = INVALID;
		this->started[i] = 0;
	}
}

MovementQuadruple Interpolator::calibrate(int id, std::list<MovementQuadruple> sentQuadruples)
{
	long int currentTime = getNanoTime();
	//if( this->northeast[0]==INVALID && this->northeast[1]==INVALID && started[id] == 0)
	if( this->started[id] == 0)
	{
		this->started[id] = currentTime;
	}
	MovementQuadruple newMovement = sentQuadruples.back();
	double diff = 3.0f;
	long int timeDiff1 = 1500000000;
	long int timeDiff2 = 2500000000;

	if( this->started[id] > currentTime + timeDiff2 )
	{
		newMovement.setTimestamp( currentTime );
		newMovement.setRollPitchYawrate( 0, 0, 0 );
	}
	else if( this->started[id] > currentTime + timeDiff1 )
	{
		newMovement.setTimestamp( currentTime );
		newMovement.setRollPitchYawrate( -diff, -diff, 0 );
	}
	else
	{
		newMovement.setTimestamp( currentTime );
		newMovement.setRollPitchYawrate( diff, diff, 0 );
	}

	return newMovement;
}

MovementQuadruple Interpolator::calculateNextMQ(std::list<MovementQuadruple> sentQuadruples, std::list<Position6DOF> positions, Position6DOF target, int id)
{

	/* Nothing has been sent so far. */
	if( sentQuadruples.size() == 0 )
	{
		return MovementQuadruple(THRUST_START, 0, 0, 0);
	}

	/* A MovementQuadruple has been sent before. */
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
	int size = positions.size();
	double deltaTarget[size];	// Absolute distance to latest target
	double deltaAbsPosition[size-1];	// equals speed	/* arraysize FIXME */
	double deltaSpeed[size-2];	// equals acceleration	/* arraysize FIXME */
	int counter = 0;
	Position6DOF positionA, positionB;	// positionA is older than positionB

	/* Calculate values for declared arrays above for later usage. */
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
					//oscillate = true;
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
	if( counter > 1 )	// Enough data to calculate new thrust value (at least two values)
	{
		/*positionA.setPosition( positions.back().getPosition() );
		positionA.getTimestamp( positions.back().getTimestamp() );*/
		double zDistanceA = positionA.getDistanceZ( target );
		double zDistanceB = positionB.getDistanceZ( target );
		double timediffAB = positionB.getTimestamp() - positionA.getTimestamp();
		double timediffNormalized = (double) timediffAB / 1000000000;	// should be in seconds
		double absDistanceAB = positionA.getAbsoluteDistance( positionB );
		double newThrust = newMovement.getThrust() + calculateThrustDiff(zDistanceA, zDistanceB, absDistanceAB, timediffNormalized);
		newMovement.setThrust( newThrust ); 
	}

	/* Calculate rest */
	if( counter > 1 )	// Enough data to calculate new rpy values (at least two values)
	{
		float newRoll = newMovement.getRoll();
		float newPitch = newMovement.getPitch();
		float newYawrate = newMovement.getYawrate();
		
		//newRoll += calculatePlaneDiff();	//TODO
		//newPitch += calculatePlaneDiff();	//TODO
		newMovement.setRollPitchYawrate(newRoll, newPitch, newYawrate);
	}
	
	return newMovement;
}

int calculateThrustDiff( double zDistanceFirst, double zDistanceLatest, double absDistanceFirstLatest, double timediffNormalized )
{
	int newThrust = 0;
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

float calculatePlaneDiff( double aDistanceFirst, double aDistanceLatest, double absDistanceFirstLatest, double timediffNormalized, double aSentLatest ) 
{

	float diff = 0;
	double aAbsDistance = abs(aDistanceFirst-aDistanceLatest);
	double distanceFactor = 0.1 + fmin(2.0, (atan(aAbsDistance*1000.0)+1.0)); // higher if further from target, between [0, 2]	//TODO

	/* Difference calculated as a-speed in mm/s. 
	 Positive if going in normalized positive direction. */
	double aSpeed = (aDistanceFirst-aDistanceLatest) * timediffNormalized;	// in mm/s
	bool distanceIncrease = false;
	
	/*
	 * Do not change value if
	 * 	speed is right and right direction
	 * 	close to target and right direction
	 * Increase abs(value) if
	 * 	too slow
	 * Decrease abs(value) if
	 * 	faster than min-speed, close to target and right direction
	 * 	too fast
	 * Negate value if
	 * 	going in wrong direction
	 */	
	/* 
	 * TODO if too slow, SPEED_MIN_PLANE needs to be changed
	 */
	
	// right direction: (aSpeed>0 && aDistanceLatest>0) 
	// close to target: abs(aDistanceLatest)<DISTANCE_CLOSE_TO_TARGET
	if( (aSpeed>0 && aDistanceLatest>0) && (aSpeed<SPEED_MIN_PLANE))
	{
		diff += ROLL_STEP; 
	}
	else if( -aSpeed>SPEED_MAX_PLANE )
	{
		diff += ROLL_STEP; 
	}
	else if( (-aSpeed>SPEED_MIN_PLANE) && (aDistanceLatest<0) && (abs(aDistanceLatest)<DISTANCE_CLOSE_TO_TARGET) )
	{
		diff += ROLL_STEP; 
	}
	else if( (aSpeed<0 && aDistanceLatest<0) && (-aSpeed<SPEED_MIN_PLANE) )
	{
		diff -= ROLL_STEP; 
	}
	else if( aSpeed>SPEED_MAX_PLANE )
	{
		diff -= ROLL_STEP; 
	}
	else if( (aSpeed>SPEED_MIN_PLANE) && (aDistanceLatest>0) && (abs(aDistanceLatest)<DISTANCE_CLOSE_TO_TARGET) )
	{
		diff -= ROLL_STEP; 
	}

	if( diff != 0 )
	{
		return diff;
	}

	if( aAbsDistance
	
	return diff;
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

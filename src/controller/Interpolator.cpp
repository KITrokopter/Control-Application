#include "Interpolator.hpp"
//#include "InterpolatorInfo.hpp"
#include "Controller.hpp"

unsigned int calculateThrustDiff( float zDistanceFirst, float zDistanceLatest, float absDistanceFirstLatest, double timediffNormalized );
float calculatePlaneDiff( double aDistanceFirst, double aDistanceLatest, double absDistanceFirstLatest, double timediffNormalized, double aSentLatest );
bool negativeRotationalSign( double rotation, Position6DOF pos, Position6DOF target );
MovementQuadruple calculateRollPitch( double rotation, Position6DOF pos, Position6DOF target );
static bool closeToTarget( Position6DOF position1, Position6DOF position2, double range );

Interpolator::Interpolator()
{
	this->stepSizeOfChange = 1;
	this->aTimeSwitch = 0;
	for( int i = 0; i < MAX_NUMBER_QUADCOPTER_HIGH; i++ )
	{
		this->status[i] = InterpolatorInfo();
	}
	timeDiff1 = 0;
	timeDiff2 = 0;
	timeDiff3 = 0;
}

/*
MovementQuadruple Interpolator::calibrate(int id, std::list<MovementQuadruple> sentQuadruples)
{
	// TODO
	long int currentTime = getNanoTime();
	MovementQuadruple newMovement = sentQuadruples.back();
	return newMovement;
}
*/


MovementQuadruple Interpolator::calculateNextMQ(std::list<MovementQuadruple> &sentQuadruples, std::list<Position6DOF> &positions, Position6DOF &target, int id)
{
	/*if( TEST_ROLL_PITCH )
	{
		long int aTime = getNanoTime();
		if( (aTime/5000000000)%2 == 0 )
		{
			if( aTimeSwitch==1 )
			{
				ROS_INFO("ROLL_MAX sent");
			}
			aTimeSwitch = 0;
			return MovementQuadruple(THRUST_START, ROLL_MAX, 0, 0);
		} 
		else
		{
			if( aTimeSwitch==0 )
			{
				ROS_INFO("PITCH_MAX sent");
			}
			aTimeSwitch = 1;
			return MovementQuadruple(THRUST_START, 0, PITCH_MAX, 0);
		}
	}*/
	//ROS_INFO("interpolate 01 calculateNextMQ");
	long int currentTime = getNanoTime();
	MovementQuadruple newMovement = MovementQuadruple(THRUST_START, 0, 0, 0); // Nothing has been sent so far

	if( sentQuadruples.size() > 0 )
	{
		newMovement = sentQuadruples.back();	// A MovementQuadruple has been sent before
	}
	newMovement.setTimestamp( currentTime );

	checkState( id );
	//ROS_INFO("interpolate 02 after checkState");
	switch( this->status[id].getState() )
	{
		case UNSTARTED:
			ROS_INFO("interpolate 03a unstarted");
			ROS_INFO("Error in switch - calculateNextMQ.");	// FIXME ROS_ERROR ?
			newMovement.setThrust( THRUST_MIN );
			newMovement.setRollPitchYawrate( 0, 0, 0 );
			return newMovement;
			//break;
		case STARTED:
			ROS_INFO("interpolate 03b started");
			if( this->status[id].getStarted()+timeDiff1 < currentTime )
			{
				newMovement.setRollPitchYawrate( -ROLL_MAX, -PITCH_MAX, 0 );
			}
			else
			{
				newMovement.setRollPitchYawrate( ROLL_MAX, PITCH_MAX, 0 );
			}
			return newMovement;
			//break;
		case CALC:
			//ROS_INFO("interpolate 03c calc");
			if( positions.size() > 2 )	// Enough data to calculate new rpy values (at least two values)
			{
				newMovement.setRollPitchYawrate( 0, 0, 0 );
				Position6DOF pos;
				int counter = 0;
				for(std::list<Position6DOF>::iterator it = positions.begin(); it != positions.end(); ++it)
				{
					pos.setTimestamp( it->getTimestamp() );
					if( pos.getTimestamp() > status[id].getStarted() + timeDiff1 )
					{
						pos.setPosition( it->getPosition() );
						double diffX = pos.getPosition()[0] - target.getPosition()[0];
						double diffY = pos.getPosition()[1] - target.getPosition()[1];
						double absDistance = sqrt( diffX*diffX + diffY*diffY ); // TODO check for error
						diffX = diffX / absDistance;
						diffY = diffY / absDistance;
						this->status[id].setRotation( 0 ); //acos( diffY ) );	// FIXME check
						this->status[id].setNegativeSign( false ); //negativeRotationalSign(this->status[id].getRotation(), pos, target ) );	// FIXME check
						this->status[id].setLastUpdated( currentTime );
						break;
					}
					counter++;
				}
				this->status[id].setState( DONE );
			}
			else
			{
				// TODO Error, shouldn't have happened after that time (timediff2)
			}
			newMovement.setRollPitchYawrate( 0, 0, 0 );
			return newMovement;
		default:			
			break;
	}
	
	if( this->status[id].getState() < DONE )
	{
			ROS_ERROR("Error in second switch - calculateNextMQ.");	// FIXME ROS_ERROR ?
			return newMovement;
	}
	//ROS_INFO("interpolate 05 now in DONE at time %ld", currentTime);

	/* Now in state "DONE" */
	if( (this->status[id].getStarted()+timeDiff3) >= currentTime )
	{
		ROS_INFO("interpolate 05b in if");
		/* Wait some more before starting to stabilize */
		newMovement.setRollPitchYawrate( 0, 0, 0 );
		return newMovement;
	}
	//ROS_INFO("interpolate 06");
	if( sentQuadruples.size() < 3 || positions.size() < 3 )
	{
		/* Might not get enough data from camera in a certain time.
		 * Depends probably on how fast the QC will leave the tracking area again. */
		ROS_ERROR("Not enough data in calculateNextMQ, some assumption is wrong..."); // FIXME error not info
		return newMovement;
	}
	//ROS_INFO("interpolate 07");

	/*
	 * Calculate with given calibration data, actually
	 * trying to "stabilize" now
	 */

	//ROS_INFO("interpolate 08 Enough data in calculateNextMQ, start calculation.");
	/* Save latest Position and before-latest Position */
	Position6DOF positionPast;
	Position6DOF positionNow;	// positionPast is older than positionNow
	Position6DOF posAssumed;
	std::list<Position6DOF>::iterator it = positions.end();
	int counter = 0;
	while( (it!=positions.begin()) && (counter<2) )
	{
		positionPast.setOrientation( it->getOrientation() );
		positionPast.setPosition( it->getPosition() );
		positionPast.setTimestamp( it->getTimestamp() );
		if( counter == 0 )
		{
			positionNow = positionPast;
		}
		--it;
		counter++;
	}
		// FIXME the following is an alternative to the previous while-construct. Both untested.
	/*
		while( (it!=positions.begin()) && (counter<2) )
		{
			if( counter == 1 )
			{
				positionPast.setOrientation( it->getOrientation() );
				positionPast.setPosition( it->getPosition() );
				positionPast.setTimestamp( it->getTimestamp() );
				positionNow = positions.front();
			}
			--it;
			counter++;
		}
	*/
	//ROS_INFO("interpolate 09 Got positionPast and positionNow.");

	/* Calculate predicted actual position */
	// TODO check if positions.back() isn't influenced
	posAssumed = positions.back();
	posAssumed.predictNextPosition( positionPast, PREDICT_FUTURE_POSITION_TIME );
	//ROS_INFO("interpolate 10 calculated assumedPos");


	/* Calculate thrust value - always */
	float zDiffPast = positionPast.getDistanceZ( target );	// unnecessary if prediction works
	float zDiffNow = positionNow.getDistanceZ( target );
	float zDiffAssumed = posAssumed.getDistanceZ( target );
	// unnecessary if prediction works, leave for testing
/*	double timediffPastNow = positionNow.getTimestamp() - positionPast.getTimestamp();
	double timediffNormalized = (double) timediffPastNow / 1000000000;	// should be in seconds
	double absDistancePastNow = positionPast.getAbsoluteDistance( positionNow );
	unsigned int newThrust = newMovement.getThrust() + calculateThrustDiff(zDiffPast, zDiffNow, absDistancePastNow, timediffNormalized);*/
	double timediffNowAssumed = posAssumed.getTimestamp() - positionNow.getTimestamp();
	double timediffNormalized = (double) timediffNowAssumed / ((double) 1000000000);	// should be in seconds
	float absDistanceNowAssumed = positionNow.getAbsoluteDistance( posAssumed );
	unsigned int newThrust = newMovement.getThrust() + calculateThrustDiff(zDiffNow, zDiffAssumed, absDistanceNowAssumed, timediffNormalized);
	newMovement.setThrust( newThrust );
	//ROS_INFO("interpolate 11 thrustdiff %u", newThrust);

	/* Calculate new rpy-values every MIN_TIME_TO_WAIT nanoseconds */
	if( this->status[id].getLastUpdated()-currentTime < MIN_TIME_TO_WAIT )
	{
		//ROS_INFO("interpolate 12 Do not change rpy-values, movement of sent values need to be visible.");
		return newMovement;
	}


	 /* Calculate new calibration (due to yaw-movement, if |roll|,|pitch| were high enough) */
	if( ROTATIONAL_CORRECTION )
	{
		/*
		 * TODO
		 * sent |roll|+|pitch| = change
		 * change > some threshold?
		 */
	}
	//ROS_INFO("interpolate 12 rotational correction done");

	/* Calculate correction (calibration data, predictedPosition, target) */
	MovementQuadruple rpyMovement = calculateRollPitch( status[id].getRotation(), posAssumed, target );
	newMovement.setRollPitchYawrate( rpyMovement );
	//ROS_INFO("interpolate 13 rpyMovement calculated");
	this->status[id].setLastUpdated( currentTime );

	return newMovement;

//	int size = positions.size();
//	double deltaTarget[size];	// Absolute distance to latest target
//	double deltaAbsPosition[size-1];	// equals speed	/* arraysize FIXME */
//	//double deltaSpeed[size-2];	// equals acceleration	/* arraysize FIXME */
//	int counter = 0;
//	this->status[id].setLastUpdated( currentTime );
//
//	/* Calculate values for declared arrays above for later usage. */
//	for(std::list<Position6DOF>::iterator it = positions.begin(); it != positions.end(); ++it)
//	{
//		positionPast.setOrientation( it->getOrientation() );
//		positionPast.setPosition( it->getPosition() );
//		positionPast.setTimestamp( it->getTimestamp() );
//		deltaTarget[counter] = positionPast.getAbsoluteDistance( target );
//		if( counter > 0 )
//		{
//			//deltaAbsPosition[counter-1] = positionB.getAbsoluteDistance( positionA );
//			if( counter > 1 )
//			{
//				double speedDelta = deltaAbsPosition[counter-1] - deltaAbsPosition[counter-2];
//				if( speedDelta < 0 )
//				{
//					//oscillate = true;
//					speedDelta = -speedDelta;
//				}
//				double timeDelta = positionNow.getTimestamp() - positionPast.getTimestamp();
//				//deltaSpeed[counter-2] = speedDelta / timeDelta;
//			}
//		}
//		if( positionPast.getTimestamp() != positions.back().getTimestamp() )
//		{
//			positionNow = positionPast;
//		}
//		counter++;
//	}
}

MovementQuadruple Interpolator::calculateHold(std::list<MovementQuadruple> &sentQuadruples, std::list<Position6DOF> &positions, int id)
{
	long int current = getNanoTime();
	if( this->status[id].getState() < HOLD )
	{
		this->status[id].setState( HOLD );
		this->status[id].setShutdownStarted( current );
	}

	MovementQuadruple newMovement = sentQuadruples.back();
	if( this->status[id].getState() == HOLD )
	{
		if( current > this->status[id].getShutdownStarted() )
		{
			this->status[id].setState( SHUTDOWN );
			return MovementQuadruple( 0, 0, 0, 0 );
		}
		/* 
		 * Now calculate values:
		 * mirrored at getShutdownStarted-timestamp and negated sentQuadruples 
		 */
		newMovement.invertRollPitchYawrate( HOLD_FACTOR_THRUST, HOLD_FACTOR_RPY );
	}
	if( this->status[id].getState() == SHUTDOWN )
	{
		return MovementQuadruple( 0, 0, 0, 0 );
	}
//	return newMovement;
	return MovementQuadruple( 0, 0, 0, 0 );
}


unsigned int calculateThrustDiff( float zDistanceFirst, float zDistanceLatest, float absDistanceFirstLatest, double timediffNormalized )
{
	unsigned int newThrustDiff = 0;
	float distanceFactor = 0.5; // higher if further from target, between [0, 1]	//TODO
	float threshold = 0;	// higher if timediff is higher and 	//TODO

	/* Height-difference calculated as z-speed in mm/s. Positive if inclining. */
	float zSpeed = (zDistanceFirst-zDistanceLatest) * timediffNormalized;	// in mm/s
	
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
	 */
	
	if( abs(zDistanceLatest) < DISTANCE_CLOSE_TO_TARGET ) 
	{
		ROS_ERROR("Thrustdiff is zero");
		return newThrustDiff;
	} else
	{
		ROS_ERROR("zSpeed: %f, zDistF: %f, zDistL: %f", zSpeed, zDistanceFirst, zDistanceLatest);
		if((zSpeed>0 && zSpeed<SPEED_MIN_INCLINING) || (zSpeed<SPEED_MAX_DECLINING) || (zDistanceLatest>0 && zDistanceLatest>zDistanceFirst && zSpeed<0)) 
		{  
			ROS_ERROR(" Thrustdiff increase");
			newThrustDiff += THRUST_STEP;
		}
		if((zSpeed>SPEED_MAX_INCLINING) || (zSpeed<0 && zSpeed>SPEED_MIN_DECLINING) || (zDistanceLatest<0 && zDistanceLatest<zDistanceFirst && zSpeed>0)) 
		{  
			ROS_ERROR(" Thrustdiff decrease");
			newThrustDiff -= THRUST_STEP;
		}
		return newThrustDiff;
	}
}

void Interpolator::checkState( int id )
{
	long int currentTime = getNanoTime();
	switch( this->status[id].getState() )
	{
		case UNSTARTED:
			this->status[id].setState( STARTED );
			this->status[id].setStarted( currentTime );
			break;			
		case STARTED:
			if( currentTime > this->status[id].getStarted()+timeDiff2 )
			{
				this->status[id].setState( CALC );
			}
			break;
		case CALC:
			break;
		case DONE:
			break;
		default:			
			break;
	}
	/*	if( this->status[id] == UNSTARTED )*/
}

bool negativeRotationalSign( double rotation, Position6DOF pos, Position6DOF target )
{
	/* values of the Matrix, mij is value of i. line and j. column  */
	double m11 = cos( rotation );
	double m12 = -sin( rotation );
	double m21 = sin( rotation );
	double m22 = cos( rotation );
	double *current = pos.getPosition();
    double v1 = m11 * current[0] + m12 * current[1];
    double v2 = m21 * current[0] + m22 * current[1];
    double error = sqrt((v1-current[0])*(v1-current[0]) + (v2-current[1])*(v2-current[1]));
    if( error < 0.5 )
    {
    	return false;
    }
    return true;
}

MovementQuadruple calculateRollPitch( double rotation, Position6DOF pos, Position6DOF target )	
{
	/* values of the Matrix, mij is value of i. line and j. column  */
	double m11 = cos( rotation );
	double m12 = -sin( rotation );
	double m21 = sin( rotation );
	double m22 = cos( rotation );
	double *current = pos.getPosition();
    double v1 = m11 * current[0] + m12 * current[1];
    double v2 = m21 * current[0] + m22 * current[1];
    double factor = sqrt(v1*v1 + v2*v2);
    v1 = v1 / factor;
    v2 = v2 / factor;
    double newRoll = v1 * ROLL_MAX;
    double newPitch = v2 * PITCH_MAX;
    double newYawrate = 0;
    if( closeToTarget( pos, target, RANGE_STABLE ) )
    {
        double newRoll = newRoll / 2;
        double newPitch = newPitch / 2;
    }
    return MovementQuadruple( 0, newRoll, newPitch, newYawrate );
}

float calculatePlaneDiff( double aDistanceFirst, double aDistanceLatest, double absDistanceFirstLatest, double timediffNormalized, double aSentLatest ) 
{

	/*float diff = 0;
	double aAbsDistance = abs(aDistanceFirst-aDistanceLatest);
	double distanceFactor = 0.1 + fmin(2.0, (atan(aAbsDistance*1000.0)+1.0)); // higher if further from target, between [0, 2]	//TODO
*/
	/* Difference calculated as a-speed in mm/s. 
	 Positive if going in normalized positive direction. */
	/*double aSpeed = (aDistanceFirst-aDistanceLatest) * timediffNormalized;	// in mm/s
	bool distanceIncrease = false;
	*/
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
	/*if( (aSpeed>0 && aDistanceLatest>0) && (aSpeed<SPEED_MIN_PLANE))
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
	
	return diff;*/
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

static bool closeToTarget( Position6DOF position1, Position6DOF position2, double range )
{
	double distance = position1.getAbsoluteDistance( position2 );
	if( distance < range )
	{
		return true;
	}
	return false;
}


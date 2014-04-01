#include "Interpolator.hpp"
#include "Controller.hpp"

unsigned int calculateThrustDiff( float zDistanceFirst, float zDistanceLatest, float absDistanceLatestTarget, double timediffNormalized, QuadcopterThrust thrustInfo );
bool negativeRotationalSign( double rotation, Position6DOF pos, Position6DOF target );
MovementQuadruple calculateRollPitch( double rotation, Position6DOF pos, Position6DOF target );
static bool closeToTarget( Position6DOF position1, Position6DOF position2, double range );
float calculateDistanceFactor( float distance );
float calculateDistanceFactorRPY( float distance );

Interpolator::Interpolator()
{
	this->stepSizeOfChange = 1;
	this->aTimeSwitch = 0;
	for( int i = 0; i < MAX_NUMBER_QUADCOPTER; i++ )
	{
		this->status[i] = InterpolatorInfo();
	}
 	this->timeDiff1 = 0;
	this->timeDiff2 = 0;
	this->timeDiff3 = 0;
}

MovementQuadruple Interpolator::calculateNextMQ(std::list<MovementQuadruple> &sentQuadruples, std::list<Position6DOF> &positions, Position6DOF &target, QuadcopterThrust thrustInfo, int id)
{
	long int currentTime = getNanoTime();
	MovementQuadruple newMovement = MovementQuadruple(thrustInfo.getStart(), 0, 0, 0); // Nothing has been sent so far

	if( sentQuadruples.size() > 0 )
	{
		newMovement = sentQuadruples.back();	// A MovementQuadruple has been sent before
	}
	newMovement.setTimestamp( currentTime );

	int counter = 0;
	checkState( id );
	switch( this->status[id].getState() )
	{
		case UNSTARTED:
			ROS_ERROR("Error in switch - calculateNextMQ.");
			newMovement.setThrust( THRUST_OFF );
			newMovement.setRollPitchYawrate( 0, 0, 0 );
			return newMovement;
		case STARTED:
			if( this->status[id].getStarted()+timeDiff1 < currentTime )
			{
				newMovement.setRollPitchYawrate( -ROLL_MAX, -PITCH_MAX, 0 );
			}
			else
			{
				newMovement.setRollPitchYawrate( ROLL_MAX, PITCH_MAX, 0 );
			}
			return newMovement;
		case CALC:
			{
			//ROS_INFO("interpolate 03c calc");
			newMovement.setRollPitchYawrate( 0, 0, 0 );
			Position6DOF pos;
			counter = 0;
			for(std::list<Position6DOF>::iterator it = positions.begin(); it != positions.end(); ++it)
			{
				pos.setTimestamp( it->getTimestamp() );
				if( pos.getTimestamp() > status[id].getStarted() + timeDiff1 )
				{
					pos.setPosition( it->getPosition() );
					double diffX = pos.getPosition()[0] - target.getPosition()[0];
					double diffY = pos.getPosition()[1] - target.getPosition()[1];
					double absDistance = sqrt( diffX*diffX + diffY*diffY ); // TODO check for error
					if( absDistance == 0)
					{
						ROS_ERROR("absDistance is zero");
					}
					else
					{
						diffX = diffX / absDistance;
						diffY = diffY / absDistance;
					}
					this->status[id].setRotation( 0 ); //acos( diffY ) );	// FIXME check
					this->status[id].setNegativeSign( false ); //negativeRotationalSign(this->status[id].getRotation(), pos, target ) );	// FIXME check
					this->status[id].setLastUpdated( currentTime );
					break;
				}
				counter++;
			}
			this->status[id].setState( DONE );
			newMovement.setRollPitchYawrate( 0, 0, 0 );
			return newMovement;
			}break;
		default:			
			break;
	}
	
	if( this->status[id].getState() < DONE )
	{
			ROS_ERROR("Error in second switch - calculateNextMQ.");
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
	if( sentQuadruples.size() < 3 || positions.size() < 3 )
	{
		/* Might not get enough data from camera in a certain time.
		 * Depends probably on how fast the QC will leave the tracking area again. */
		ROS_ERROR("Not enough data in calculateNextMQ, some assumption is wrong...");
		return newMovement;
	}

	/*
	 * Calculate with given calibration data, actually
	 * trying to "stabilize" now
	 */

	//ROS_INFO("interpolate 08 Enough data in calculateNextMQ, start calculation.");
	/* Save latest Position and before-latest Position */
	Position6DOF positionPast;
	Position6DOF positionNow = positions.back();	// positionPast is older than positionNow
	Position6DOF posAssumed;
	std::list<Position6DOF>::reverse_iterator rit;
	counter = 0;
	for( rit=positions.rbegin(); (rit!=positions.rend()) && (counter<2); ++rit )
	{
		if( counter == 1 )
		{
			positionPast.setOrientation( rit->getOrientation() );
			positionPast.setPosition( rit->getPosition() );
			positionPast.setTimestamp( rit->getTimestamp() );
		}
		counter++;
	}
		// the following is an alternative to the previous while-construct.
	/*
		while( (it!=positions.begin()) && (counter<2) )
		{
			if( counter == 1 )
			{
				positionPast.setOrientation( it->getOrientation() );
				positionPast.setPosition( it->getPosition() );
				positionPast.setTimestamp( it->getTimestamp() );
				positionNow = positions.back();
			}
			--it;
			counter++;
		}
	*/
	//ROS_INFO("interpolate 09 Got positionPast and positionNow.");

	/* Calculate predicted actual position */
	posAssumed = positions.back();
	posAssumed.predictNextPosition( positionPast, PREDICT_FUTURE_POSITION_TIME );
	//ROS_INFO("interpolate 10 calculated assumedPos");


	/* Calculate thrust value - always */
	float zDiffPast = positionPast.getDistanceZ( target );	// unnecessary if prediction works
	float zDiffNow = positionNow.getDistanceZ( target );
	float zDiffAssumed = posAssumed.getDistanceZ( target );
	// leave for testing
/*	double timediffPastNow = positionNow.getTimestamp() - positionPast.getTimestamp();
	double timediffNormalized = (double) timediffPastNow / 1000000000;	// should be in seconds
	double absDistancePastNow = positionPast.getAbsoluteDistance( positionNow );
	unsigned int newThrust = newMovement.getThrust() + calculateThrustDiff(zDiffPast, zDiffNow, absDistancePastNow, timediffNormalized);*/
	double timediffNowAssumed = posAssumed.getTimestamp() - positionNow.getTimestamp();
	double timediffNormalized = ((double) timediffNowAssumed) / ((double) 1000000000);	// should be in seconds
	float absDistanceNowAssumed = positionNow.getAbsoluteDistance( posAssumed );
	float absDistanceNowTarget = positionNow.getAbsoluteDistance( target );
	//ROS_INFO("timediffNormalized: %f", timediffNormalized);
	unsigned int oldThrust = newMovement.getThrust();
	unsigned int newThrust = newMovement.getThrust() + calculateThrustDiff(zDiffNow, zDiffAssumed, absDistanceNowTarget, timediffNormalized, thrustInfo);
	newThrust = thrustInfo.checkAndFix( newThrust );
	//ROS_INFO("zDiffAssumed-zDiffNow: %f, old thrust %i, new thrust %i", zDiffAssumed-zDiffNow, oldThrust, newThrust);
	newMovement.setThrust( newThrust );
	//ROS_INFO("interpolate 11 thrustdiff %u", newThrust);


	/* Calculate new rpy-values every MIN_TIME_TO_WAIT nanoseconds */
/*	if( this->status[id].getLastUpdated()-currentTime < MIN_TIME_TO_WAIT )
	{
		ROS_INFO("interpolate 12 Do not change rpy-values, movement of sent values need to be visible.");
		return newMovement;
	}
*/
	 /* Calculate new calibration (due to yaw-movement, if |roll|,|pitch| were high enough) */
	if( ROTATIONAL_CORRECTION )
	{
		/*
		 * TODO
		 * sent |roll|+|pitch| = change
		 * change > some threshold?
		 */
		
	}
	//ROS_INFO("interpolate 12b rotational correction done");

	/* Calculate correction (calibration data, predictedPosition, target) */
	MovementQuadruple rpyMovement = calculateRollPitch( status[id].getRotation(), posAssumed, target );
	newMovement.setRollPitchYawrate( rpyMovement );
	//ROS_INFO("interpolate 13 rpyMovement calculated");
	this->status[id].setLastUpdated( currentTime );

	return newMovement;
}

MovementQuadruple Interpolator::calculateHold(std::list<MovementQuadruple> &sentQuadruples, std::list<Position6DOF> &positions, QuadcopterThrust thrustInfo, int id)
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


/*
 * Increase thrust if
 * 	below target, zSpeed negative
 * 	below target, zSpeed positive and too slow
 * 	above target, zSpeed negative and too high
 * Decrease thrust if
 * 	above target, zSpeed positive
 * 	above target, zSpeed negative and too slow
 * 	below target, zSpeed positive and too high
 * Do not change thrust if
 * 	other
 */
unsigned int calculateThrustDiff( float zDistanceFirst, float zDistanceLatest, float absDistanceLatestTarget, double timediffNormalized, QuadcopterThrust thrustInfo )
{
	unsigned int newThrustDiff = 0;
	float distanceFactor = calculateDistanceFactor( absDistanceLatestTarget );	

	/* Height-difference calculated as z-speed in mm/s. Positive if inclining. */
	float zSpeed = (zDistanceFirst-zDistanceLatest) / timediffNormalized;	// in mm/s
	ROS_INFO("zSpeed %f", zSpeed);
	
	double cyclesPerSecond = ((double) 1000000000) / ((double) TIME_MIN_CALC);
	double thrustStepA = ((double) THRUST_STEP) * ((double) distanceFactor) * sqrt(1/cyclesPerSecond);
	unsigned int thrustStep = thrustStepA;
	ROS_ERROR("thrustStep %i", thrustStep);
	//ROS_ERROR("absDistanceLatestTarget %f, distanceFactor %f", absDistanceLatestTarget, distanceFactor);
	//ROS_ERROR("thrustStepA %f, thrustStep %i", thrustStepA, thrustStep);
	//ROS_ERROR("cycles %f, thrustStepA %f, thrustStep %i", cyclesPerSecond, thrustStepA, thrustStep);	
	//ROS_INFO("zSpeed: %f, zDistF: %f, zDistL: %f", zSpeed, zDistanceFirst, zDistanceLatest);
	if((zSpeed<SPEED_MAX_DECLINING))
	{
		ROS_ERROR(" Thrustdiff increase 1");
		newThrustDiff += 2*thrustStep;
	}
	else if((zDistanceLatest>0 && zSpeed<0) || (zDistanceLatest>0 && zSpeed>0 && zSpeed<SPEED_MIN_INCLINING) || (zDistanceLatest<0 && zSpeed<0 && zSpeed<SPEED_MAX_DECLINING))
	{
		ROS_ERROR(" Thrustdiff increase 2");
		newThrustDiff += thrustStep;
	}
	
	if((zDistanceLatest<0 && zSpeed>0) || (zDistanceLatest<0 && zSpeed<0 && zSpeed>SPEED_MIN_DECLINING) || (zDistanceLatest<0 && zSpeed>0 && zSpeed>SPEED_MAX_INCLINING))
	{
		ROS_ERROR(" Thrustdiff decrease");
		newThrustDiff -= thrustStep;
	}
	return newThrustDiff;
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
	if(factor == 0 )
	{
		ROS_ERROR("Factor is zero");
	}
	else
	{
		v1 = v1 / factor;
		v2 = v2 / factor;
	}
	double distanceXY = pos.getAbsoluteDistanceXY( target );
	double factorDistance = calculateDistanceFactorRPY( distanceXY );
	double newRoll = v1 * ROLL_MAX * factorDistance;
	double newPitch = v2 * PITCH_MAX * factorDistance;
	double newYawrate = 0;
	ROS_ERROR("roll %f, pitch %f, distXY %f, facDist %f", newRoll, newPitch, distanceXY, factorDistance);
	
	if( closeToTarget( pos, target, RANGE_STABLE ) )
	{
		double newRoll = newRoll / 2;
		double newPitch = newPitch / 2;
	}
	return MovementQuadruple( 0, newRoll, newPitch, newYawrate );
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

float calculateDistanceFactor( float distance )
{
	distance = abs( distance );
	if( distance <= ((float) DISTANCE_CLOSE) )
	{
		return 0.0;
	}
	else if( distance < ((float) DISTANCE_HIGH) )
	{
		float x = distance - ((float) DISTANCE_CLOSE);
		if( x == 0.0 )
		{
			return x;
		}
		return sqrt(x / (((float) DISTANCE_HIGH)-((float) DISTANCE_CLOSE)));
	} 
	else
	{
		return 1.0;
	}
}

float calculateDistanceFactorRPY( float distance )
{
	distance = abs( distance );
	if( distance <= ((float) DISTANCE_CLOSE_RPY) )
	{
		return 0.0;
	}
	else if( distance < ((float) DISTANCE_HIGH_RPY) )
	{
		float x = distance - ((float) DISTANCE_CLOSE_RPY);
		if( x == 0.0 )
		{
			return x;
		}
		return sqrt(x / (((float) DISTANCE_HIGH_RPY)-((float) DISTANCE_CLOSE_RPY)));
	} 
	else
	{
		return 1.0;
	}
}


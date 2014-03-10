#include "Controller.hpp"

bool closeToTarget( Position6DOF position1, Position6DOF position2 );

void* startThreadCalculateMovement(void* something)
{
	Controller *someOther = (Controller *) something; 
	someOther->calculateMovement();
}

void* startThreadBuildFormation(void* something)
{
	Controller *someOther = (Controller *) something; 
	someOther->buildFormation();
}

Controller::Controller()
{
	/**
  	 * NodeHandle is the main access point to communications with the ROS system.
	 * The first NodeHandle constructed will fully initialize this node, and the last
     * NodeHandle destructed will close down the node.
     */
    ros::NodeHandle n;
	//Subscriber
	//Subscriber for the MoveFormation data of the Quadcopters (1000 is the max. buffered messages)
	this->MoveFormation_sub = this->n.subscribe("MoveFormation", 1000, &Controller::MoveFormationCallback, this);
	//Subscriber for the SetFormation data of the Quadcopters (100 is the max. buffered messages)
	this->SetFormation_sub = this->n.subscribe("SetFormation", 100, &Controller::SetFormationCallback, this);
	//Subscriber for the quadcopter status data of the Quadcopters (1000 is the max. buffered messages)
	this->System_sub = this->n.subscribe("System", 1000, &Controller::SystemCallback, this);

	//Service
	//Service for BuildFormation and Shutdown
	this->BuildForm_srv  = this->n.advertiseService("BuildFormation", &Controller::startBuildFormation, this);
	this->Shutdown_srv = this->n.advertiseService("Shutdown", &Controller::shutdown, this);
	this->QuadID_srv = this->n.advertiseService("SetQuadcopters", &Controller::setQuadcopters, this);

	//Publisher
	//Publisher of Message to API
	this->Message_pub = this->n.advertise<api_application::Message>("Message", 100);
	

	//Client
	this->Announce_client = this->n.serviceClient<api_application::Announce>("Announce");
	this->Shutdown_client = this->n.serviceClient<control_application::Shutdown>("Shutdown");
	
	//All control variables are set to zero
	this->shutdownMutex.lock();
	this->shutdownStarted = false;	// shutdown not started
	this->shutdownMutex.unlock();
	
	this->receivedQCMutex.lock();
	this->receivedQuadcopters = false; // received no quadcopters
	this->receivedQCMutex.unlock();
	
	
	this->buildFormationMutex.lock();
	this->buildFormationFinished = false; // has not built formation
	this->buildFormationMutex.unlock();
	
	this->stopFormationMutex.lock();
	this->buildFormationStop = true; // buildFormation has not been stopped
	this->stopFormationMutex.unlock();
	
	this->formation = new Formation();
	
	ROS_INFO("ROS stuff set up");
	
	for(int i = 0; i< MAX_NUMBER_QUADCOPTER; i++)
	{
		//Initialize tracked (no quadcopter is tracked at the beginning)
		trackedArrayMutex.lock();
		tracked[i] = false;
		trackedArrayMutex.unlock();
	}
	ROS_INFO("Constructing done");
}

void Controller::initialize()
{
	api_application::Announce srv;
	srv.request.type = 2;
	if(Announce_client.call(srv))
	{
		this->senderID = srv.response.id;
	}
	ROS_INFO("Initialize done");
}

void Controller::updatePositions(std::vector<Vector> positions, std::vector<int> ids, std::vector<int> updates)
{
		
	//ROS_INFO("Update Position");
	if(!receivedQuadcopters || !receivedFormation)
	{
		return;
	}
	/* Save position vectors */	
	std::vector<Position6DOF> newListItem;
	int i = 0;
	int id = 0;
	for(std::vector<Vector>::iterator it = positions.begin(); it != positions.end(); ++it, i++)
	{
		id = getLocalId(i);
		//ROS_INFO("Global id is %i",i);
		//ROS_INFO("Local Id is %i", id);
		if(id == -1)
		{
			continue;	
		}
		this->lastCurrentMutex.lock();
		this->lastCurrent[id] = getNanoTime();
		Position6DOF newPosition = Position6DOF (it->getV1(), it->getV2(), it->getV3());
		newPosition.setTimestamp(this->lastCurrent[id]);
		this->lastCurrentMutex.unlock();
		newListItem.push_back( newPosition );
		
		if( it->getV1() != INVALID ) 
		{	
			//ROS_INFO("Valid");
			/* Quadcopter is tracked */
			trackedArrayMutex.lock();
			bool track = this->tracked[id];
			trackedArrayMutex.unlock();
			if( track == false )
			{
				//ROS_INFO("track false");
				/* Quadcopter has not been tracked before */
				if(this->quadcopterMovementStatus[id] == CALCULATE_START)
				{
					this->quadcopterMovementStatus[id] = CALCULATE_STABILIZE;
				}
			}
			//ROS_INFO("tracked id");
			trackedArrayMutex.lock();
			this->tracked[id] = true;
			trackedArrayMutex.unlock();
		} else
		{
			//ROS_INFO("Invaldi");
			trackedArrayMutex.lock();
			this->tracked[id] = false;
			trackedArrayMutex.unlock();
		}
		//ROS_INFO("Push back");
		this->listPositionsMutex.lock();
		this->listPositions[id].push_back( newPosition ); 
		while( this->listPositions[id].size() > 30 )
		{
			//ROS_INFO("erasing");
			// Remove oldest elements
			this->listPositions[id].erase( this->listPositions[id].begin() );
		}
		this->listPositionsMutex.unlock();
	}	
	//ROS_INFO("Update Position finished");	
}


	
/*
 * Calculates the movement vector of each quadcopter non stop and sends the vector first to convertMovement to set the yaw, pitch, roll and thrust
 * values. Then sends the movement to the quadcopter. Routine runs till shutdown formaiton is called.
 */	
void Controller::calculateMovement()
{
	ROS_INFO("Calculation started");
	int numberOfLanded = 0;
	int amount = quadcopterMovementStatus.size();
	/* As long as we are not in the shutdown process, calculate new Movement data */
	bool end;
	this->receivedFormMutex.lock();
	if(this->receivedFormation)
	{
		end = numberOfLanded >= this->formation->getAmount();
	}
	else
	{
		end = false;
	}
	this->receivedFormMutex.unlock();
	while(!end)
	{
		//ROS_INFO("Calculate");
		for(int i = 0; (i < amount) && (!end); i++)
		{	
			receivedFormMutex.lock();
			receivedQCMutex.lock();
			bool enoughData = this->receivedFormation && this->receivedQuadcopters;
			receivedFormMutex.unlock();
			receivedQCMutex.unlock();
			if(this->quadcopterMovementStatus[i] == CALCULATE_LAND || this->quadcopterMovementStatus[i] == CALCULATE_HOLD) //only for testing
			{
			if( enoughData)
			{
				checkInput();
			}
			}
			
			/* Calculation */
			double current[3];
			double target[3];
			double moveVector[3];
			for(int k = 0; k < 3; k++)
			{
				this->listTargetsMutex.lock();
				target[k] = this->listTargets[i].back().getPosition()[k];
				this->listTargetsMutex.unlock();
				this->listPositionsMutex.lock();
				current[k] = this->listPositions[i].back().getPosition()[k];
				this->listPositionsMutex.unlock();
			}
	
			switch( quadcopterMovementStatus[i] )
			{
				case CALCULATE_NONE:
					//ROS_INFO("None %i", i);
					/* 
					 * Take old values
					 * At beginning list has to be initialized with "send none".
					 * No convertMovement has to be called. 
					 */
					break;
				case CALCULATE_START:	
					//ROS_INFO("Start %i", i);
					moveUp( i );
					break;
				case CALCULATE_STABILIZE:
					ROS_INFO("Stabilize %i", i);
					/*TODO*/
					stabilize( i );
					break;
				case CALCULATE_HOLD:	
					ROS_INFO("Hold %i", i);				
					/*TODO hold and (if in shutdown) do it fast*/
					
					break;
				case CALCULATE_MOVE:
					//ROS_INFO("Move %i", i);
					moveVector[0] = target[0] - current[0];
					moveVector[1] = target[1] - current[1];
					moveVector[2] = target[2] - current[2];
					convertMovement(moveVector, i);
					break;
				case CALCULATE_LAND:
					//ROS_INFO("Land %i", i);
					land( i, &numberOfLanded );
					break;
				default:
					ROS_INFO("Default %i", i);
					moveVector[0] = INVALID;
					moveVector[1] = INVALID;
					moveVector[2] = INVALID;
					break;
			}
			sendMovementAll();
			/* Shutdown */
			landMutex.lock();
			end = landFinished;
			landMutex.unlock();
			
		}
	}	
}

/*
 * While untracked:
 * First second: thrust THRUST_START
 * Next 4 seconds: thrust THRUST_STAND_STILL
 * After that probably an error occured and we can't say where it
 * it and should shutdown.
 */
void Controller::moveUp( int internId )
{
	double moveVector[3];	
	
	stopFormationMutex.lock();
	bool continueMoveUp = buildFormationStop;
	stopFormationMutex.unlock();	
	if( continueMoveUp )
	{
		MovementQuadruple newMovement = MovementQuadruple( THRUST_START, 0, 0, 0 );
		movementAll[internId] = newMovement;
	} 
}

void Controller::stabilize( int internId )
{
	/* TODO */
	/*
	 * Delta der Position berechnen
	 * 	falls nicht möglich: sende alten Wert (return)
	 *
	 * Delta der Beschleunigung berechnen
	 *	falls nicht möglich: sende alten Wert (return) oder schätze ?
	 * 
	 * Geschwindigkeit und Beschleunigung der letzten x Male berechnen, Werte 
	 * dementsprechend für Interpolation setzen (die Raten)
	 *
	 */
	Interpolator interpolator = Interpolator();
	
	this->listPositionsMutex.lock(); 
	MovementQuadruple newMovement = interpolator.calculateNextMQ(this->listSentQuadruples[internId], this->listPositions[internId], internId);
	this->listPositionsMutex.unlock();
}

bool Controller::isStable( int internId )
{
	/* 
	 * Compare latest position of QC with 
	 * position of QC "compareTimeX"-elements before. 
	 * Assumption: 30 Elements ~ 1 sec.
	 */
	int compareTime[3] = { 1, 5, 50 };
	
	this->listPositionsMutex.unlock();
	size_t sizeOfListPositions = this->listPositions[internId].size();
	this->listPositionsMutex.unlock();
    if( sizeOfListPositions > compareTime[2] )
    {
		/* Reverse iterator to the reverse end */
		int counter = 0;
		std::list<Position6DOF>::reverse_iterator rit = this->listPositions[internId].rbegin();
		for( ; rit != this->listPositions[internId].rend(); ++rit )
		{
			if( counter == compareTime[0] )
			{
				if( !closeToTarget( listPositions[internId].back(), *rit ) )
				{
					return false;
				}
			}
			if( counter == compareTime[1] )
			{
				if( !closeToTarget( listPositions[internId].back(), *rit ) )
				{
					return false;
				}
			}
			if( counter == compareTime[2] )
			{
				if( !closeToTarget( listPositions[internId].back(), *rit ) )
				{
					return false;
				}
			}
			counter++;
		}
		return true;
    } else if ( sizeOfListPositions > compareTime[1] )
    {
		/* Possible to work with available information? */
		return false;
    } else if ( sizeOfListPositions > compareTime[0] )
    {
		return false;
    } else
    {
        /* No information to work with, start emergency routine? */
        return false;
    }
}

void Controller::hold( int internId )
{
	/* TODO */
}

void Controller::land( int internId, int * nrLand )
{
	this->trackedArrayMutex.lock();
	//Decline until crazyflie isn't tracked anymore
	if(tracked[internId] == true)
	{
		this->movementAll[internId].setThrust(THRUST_DECLINE);
	}
	else
	{
		//Shutdown crazyflie after having left the tracking area.
		this->movementAll[internId].setThrust(THRUST_MIN);
		this->quadcopterMovementStatus[internId] = CALCULATE_NONE;
		(*nrLand)++;
	}
	this->trackedArrayMutex.unlock();
}

/*
 * Checks if formation movement data and quadcopter positions have been received lately. Otherwise calls emergencyroutine.
 */
bool Controller::checkInput()
{
	//TODO What about QuadStatus? Emergency routine for transfer stop/lack?
	for(int i = 0; i < this->quadcopterMovementStatus.size(); i++)
	{
		this->receivedQCStMutex.lock();
		bool save = !this->receivedQuadStatus[i];
		this->receivedQCStMutex.unlock();
		if(save)
		{
			continue;
		}
		/* Battery */
		if(this->battery_status[i] < LOW_BATTERY && quadcopterMovementStatus[i] != CALCULATE_NONE)
		{
			std::string message("Battery of Quadcopter %i is low (below %f). Shutdown formation\n", i, LOW_BATTERY);
			ROS_INFO("Battery of Quadcopter %i is low (below %f). Shutdown formation\n", i, LOW_BATTERY);
			emergencyRoutine(message);
		}
		if(quadcopterMovementStatus[i] != CALCULATE_MOVE && quadcopterMovementStatus[i] != CALCULATE_STABILIZE)
		{
			continue;
		}
		long int currentTime = getNanoTime();
		this->lastFormationMovementMutex.lock();
		long int lastForm = this->lastFormationMovement;
		this->lastFormationMovementMutex.unlock();
		if(currentTime - lastForm > TIME_UPDATED_END)
		{
		      std::string message("No new formation movement data has been received since %i sec. Shutdown formation\n", TIME_UPDATED_END);
		      ROS_INFO("No new formation movement data has been received since %i sec. Shutdown formation\n", TIME_UPDATED_END);
		      emergencyRoutine(message);
		      return false;
		}
		this->lastCurrentMutex.lock();
		long int lastCur = this->lastCurrent[i];
		this->lastCurrentMutex.unlock();
		if(currentTime - lastCur > TIME_UPDATED_END)
		{
		      std::string message("No quadcopter position data has been received since %i sec. Shutdown formation\n", TIME_UPDATED_END);
		      ROS_INFO("No quadcopter position data has been received since %i sec. Shutdown formation\n", TIME_UPDATED_END);
		      emergencyRoutine(message);
		      trackedArrayMutex.lock();
		      tracked[i] = false;
		      trackedArrayMutex.unlock();
		      return false;
		}
		if(currentTime - lastCur > TIME_UPDATED_CRITICAL)
		{
		      trackedArrayMutex.lock();
		      tracked[i] = false;
		      trackedArrayMutex.unlock();
		      return false;
		}
	}
	
	return true;
}

/*
 * Emergency Routine. Gets started e.g. low battery status. Sends warning via Ros and then shuts down formation.
 */
void Controller::emergencyRoutine(std::string message)
{
	ROS_INFO("Emergency Routine called");
	api_application::Message msg;
	msg.senderID = this->senderID;
	//Type 2 is a warning message
	msg.type = 2;
	msg.message = message;
	this->Message_pub.publish(msg);
	shutdownMutex.lock();
	bool end = this->shutdownStarted;
	shutdownMutex.unlock();
	if(!end)
	{
		ROS_INFO("I want to shutdown");
		control_application::Shutdown srv;
		Shutdown_client.call(srv);
	}
}


/*
 * Converts vector in yaw, pitch, roll and thrust values.
 */
void Controller::convertMovement(double* const vector, int internId)
{
	/* conversion from vectors to thrust, yawrate, pitch... */
	int thrust_react_z_low = -5;
	int thrust_react_z_high = 5;
	int thrust = 0;
	MovementQuadruple * movement = &(this->movementAll[internId]);
	if (vector[2] > thrust_react_z_high) {
		thrust = movement->getThrust() + THRUST_STEP;
		movement->setThrust(thrust);
	} else if (vector[2] < thrust_react_z_high) {
		thrust = movement->getThrust() - THRUST_STEP;
		movement->setThrust(thrust);
	} else {
		/* Probably nothing to do here. */
	}

	double length = 0;
	for (int i = 0; i < 3; i++) {
		length = length + vector[i]*vector[i];
	}
	length = sqrt(length);

	double ratio_roll = vector[0] / length;
	double ratio_pitch = vector[1] / length;
	movement->setRollPitchYawrate(	ratio_roll + ROLL_STEP, ratio_pitch + PITCH_STEP, 0.0);
}

/*
 * Creates a Ros message for the movement of each quadcopter and sends this 
 * to the quadcopter modul
 */
void Controller::sendMovementAll()
{
	//ROS_INFO("sendMovementAll started");
	//Creates a message for each quadcopter movement and sends it via Ros
	control_application::quadcopter_movement msg;
	//ROS_INFO("amount %zu",this->movementAll.size());
	
	long int currentTime = getNanoTime();
	std::vector< MovementQuadruple > newListElement;
	for(int i = 0; i < movementAll.size(); i++)
	{
		if( this->quadcopterMovementStatus[i] != CALCULATE_NONE )
		{
			//ROS_INFO("%i",i);
			msg.thrust = this->movementAll[i].getThrust();
			msg.roll = this->movementAll[i].getRoll();
			msg.pitch = this->movementAll[i].getPitch();
			msg.yaw = this->movementAll[i].getYawrate();
			this->Movement_pub[i].publish(msg);		
			this->movementAll[i].setTimestamp( currentTime );

			// Save Element (TODO only if not too young)
			this->listSentQuadruples[i].push_back( this->movementAll[i] );
			if( this->listSentQuadruples[i].size() > 5 )
			{
				// Remove oldest elements
				this->listSentQuadruples[i].erase( this->listSentQuadruples[i].begin() );
			}
		}
	}
	//ROS_INFO("sendMovementAll finished");
}

/*
 * Calculates the new Targets considering the previous targets and the formation movement vector (without orientation right now)
 */
void Controller::setTargetPosition()
{
	
	long int currentTime = getNanoTime();
	Position6DOF newTarget;
	//Iterate over all quadcopters in formation and set new target considering old target and formation Movement
	for(int i = 0; i < this->listTargets.size(); i++)
	{
		this->listTargetsMutex.lock();
		Position6DOF latestTarget = this->listTargets[i].back();
		double targetOld[3];
		for(int k = 0; k < 3; k++)
		{
			targetOld[k] = latestTarget.getPosition()[k];
		}
		this->listTargetsMutex.unlock();
		double targetNew[3];
		this->formationMovementMutex.lock();
		targetNew[0] = targetOld[0] + this->formationMovement.back()[0];
		targetNew[1] = targetOld[1] + this->formationMovement.back()[1];
		targetNew[2] = targetOld[2] + this->formationMovement.back()[2];
		this->formationMovementMutex.unlock();
		//Check if new position would be in tracking area
		Vector vector = Vector(targetNew[0],targetNew[1],targetNew[2]);
		/*if(!this->trackingArea.contains(vector))
		{
			std::string message("Formation Movement is invalid. Quadcopter %i would leave Tracking Area.\n", i);
			ROS_INFO("Warning:Formation Movement is invalid. Quadcopter %i would leave Tracking Area.",i);
			emergencyRoutine(message);
			return;
		}*/ 
		//Commented because of testing
		newTarget.setPosition(targetNew);
		newTarget.setTimestamp(currentTime);
		this->listTargetsMutex.lock();
		this->listTargets[i].push_back(newTarget);
		this->listTargetsMutex.unlock();
	}
	
}

int Controller::getLocalId(int globalId)
{
  for(int i = 0; i < this->quadcopters.size() ; i++ )
  {
	if(globalId == this->quadcopters[i])
	{
	  return i;
	}
  }
  return -1;
}



/*
 * Service to set Quadcopter IDs
 */
bool Controller::setQuadcopters(control_application::SetQuadcopters::Request  &req, control_application::SetQuadcopters::Response &res)
{
	ROS_INFO("Service setQuadcopters has been called amount %i", req.amount);
	unsigned long int i;
	for( i = 0; i < req.amount; i++)
	{
		//ROS_INFO("Array %lu", req.quadcoptersId[i]);
                this->quadcopters.push_back(req.quadcoptersId[i]);
		this->quadcopterMovementStatus.push_back(CALCULATE_NONE);
		ROS_INFO("Size of MovementAll %zu", movementAll.size());
		MovementQuadruple newMoveQuad = MovementQuadruple(0, 0, 0, 0);
		this->movementAll.push_back(newMoveQuad);
		
		//Initialization of Arrays of Lists
		std::list<Position6DOF> newEmptyListPosition;
		this->listPositionsMutex.lock();
		this->listTargetsMutex.lock();
		this->receivedQCStMutex.lock();
		this->listPositions.push_back(newEmptyListPosition);
		this->listTargets.push_back(newEmptyListPosition);	      
		this->receivedQuadStatus[i] = false; // received no quadcopter status information
		this->receivedQCStMutex.unlock();
		this->listTargetsMutex.unlock();
		this->listPositionsMutex.unlock();

		std::list<MovementQuadruple> newEmptyListMovement;		
		this->listSentQuadruples.push_back(newEmptyListMovement);
		ROS_INFO("Initialization done");
		
		//Subscriber to quadcopter status
		std::stringstream topicNameQS;
		int id = this->quadcopters[i];
  		topicNameQS << "quadcopter_status_" << id;
		this->QuadStatus_sub[i] = this->n.subscribe<quadcopter_application::quadcopter_status>(topicNameQS.str().c_str(), 1000, boost::bind(&Controller::QuadStatusCallback, this, _1, i));
		ROS_INFO("QCStatus Topics have been initialized");
		//Publisher of Movement			
		std::stringstream topicNameMov;
  		topicNameMov << "quadcopter_movement_" << id;
		//Publisher for the Movement data of the Quadcopts (1000 is the max. buffered messages)
		this->Movement_pub[i] = this->n.advertise<control_application::quadcopter_movement>(topicNameMov.str().c_str(), 1000);
		ROS_INFO("QCMovement Topics have been initialized");			
	}
	receivedQCMutex.lock();
	receivedQuadcopters = true;
	receivedQCMutex.unlock();
	
	/* TODO: Error-handling. */
	pthread_create(&tCalculateMovement, NULL, startThreadCalculateMovement, this);
	ROS_INFO("Thread tCalculateMovement set up");
	
	return true;
}

/*
 * Builds Formation by starting one quadcopter after another, finding the right position and then
 * inclining a little to avoid collisions. So there is a "being tracked" and "moving" level, and a "standing still"
 * at the right position level.
 */
void Controller::buildFormation()
{
	ROS_INFO("Service buildFormation has been called");
	bool notEnoughData = true;
	do
	{	      
	      receivedQCMutex.lock();
	      receivedFormMutex.lock();
	      notEnoughData = !receivedQuadcopters || !receivedFormation ;
	      receivedFormMutex.unlock();
	      receivedQCMutex.unlock();
	      //TODO check if setquadcopters/ setformation
	} while( notEnoughData );
	//Get the formation Positions and the distance.
	int formationAmount = this->formation->getAmount();
	Position6DOF formPos[formationAmount];
	for( int i = 0; i < formationAmount; i++)
	{
		formPos[i] = this->formation->getPosition()[i];	
	}
	double distance = this->formation->getDistance();
	//Pointer to the first tracked quadcopter
	double first[3];
	//Start one quadcopter after another
	for(int i = 0; i < formationAmount; i++)
	{
		ROS_INFO("Starting QC %i",i);
		//Starting/ Inclining process
		quadcopterMovementStatus[i] = CALCULATE_START;
		//Calculate the wanted position for quadcopter i
		ROS_INFO("test1");
		//double * pos = formPos[i].getPosition();
		double pos[3];
		double target[3];
		for(int k = 0; k < 3; k++)
		{
			pos[k] = formPos[i].getPosition()[k];
			target[k] = pos[k] * distance;
		}
		/*ROS_INFO("test1,5");
		target[0] = pos[0] * distance;
		target[1] = pos[1] * distance;
		target[2] = pos[2] * distance;*/
		ROS_INFO("test2");
		//As long as the quadcopter isn't tracked, incline
		while(this->quadcopterMovementStatus[i] == CALCULATE_START)
		{
			//Wait
			//TODO When working right, do nothing here and just wait till it's tracked
			/*this->movementAll[i] = MovementQuadruple(THRUST_START, 0, 0, 0);
			ROS_INFO("Moving up");
			sendMovementAll();*/
		}
		ROS_INFO("Tracked");
		//If this is the first tracked quadcopter set it as a reference point for all the others
		if( i == 0)
		{
			ROS_INFO("First one");
			this->listPositionsMutex.lock();
			if(!listPositions[0].empty())
			{
				for(int k = 0; k < 3; k++)
				{
					first[k] = listPositions[0].back().getPosition()[k];
				}
			}
			this->listPositionsMutex.unlock();
			ROS_INFO("First set");
			Position6DOF firstElement;
			firstElement.setPosition(first);
			this->listTargetsMutex.lock();
			ROS_INFO("Push back first Element");
			this->listTargets[0].push_back(firstElement);
			this->listTargetsMutex.unlock();
		}
		else
		{
			//Set all the other positions according to the first crazyflie
			ROS_INFO("Set the others");
			target[0] += first[0];
			target[1] += first[1];
			target[2] += first[2];
			Position6DOF targetElement;
			targetElement.setPosition(target);
			this->listTargetsMutex.lock();
			this->listTargets[i].push_back(targetElement);
			this->listTargetsMutex.unlock();
			this->quadcopterMovementStatus[i] = CALCULATE_MOVE;
		}
		ROS_INFO("Inclining");
		//Incline a little bit to avoid collisions (there is a level with the qc which are already in position and a moving level)
		double pointer[3];
		this->listTargetsMutex.lock();
		//double* pointer = this->listTargets.back()[i].getPosition();
		for(int k = 0; k < 3; k++)
		{
			pointer[k] = this->listTargets[i].back().getPosition()[k];
		}
		this->listTargetsMutex.unlock();
		pointer[0] += 0;
		pointer[1] += 0;
		pointer[2] += distance;
		Position6DOF element;
		element.setPosition(pointer);
		this->listTargetsMutex.lock();
		//this->listTargets.back()[i].setPosition(pointer);
		this->listTargets[i].push_back(element);
		this->listTargetsMutex.unlock();
		this->quadcopterMovementStatus[i] = CALCULATE_MOVE;
		ROS_INFO("Done with %i",i);
	}
	ROS_INFO("BuildFormation finished");
	this->buildFormationMutex.lock();
	this->buildFormationFinished = true;
	this->buildFormationMutex.unlock();	
	
}


/*
 * Sets global variable which indicates that the build formation process is started
 */
bool Controller::startBuildFormation(control_application::BuildFormation::Request  &req, control_application::BuildFormation::Response &res)
{
	pthread_create(&tBuildFormation, NULL, startThreadBuildFormation, this);
	ROS_INFO("Thread tBuildFormation set up");
	return true;
}

/*
 * Shutdown Formation
 * Set all values to zero and thrust to minimum. Send the values and do nothing
 * after that.
 * The thread for calculating the next values (target position, etc.) does not
 * exit. 
 * TODO: Check description with actual code ...
 * 
 */
void Controller::shutdownFormation()
{
	ROS_INFO("ShutdownFormation started");	
	shutdownMutex.lock();
	this->shutdownStarted = true; /* Start shutdown process */
	shutdownMutex.unlock();
	
	/* Bring all quadcopters to a hold */	
	for(unsigned int i = 0; i < quadcopterMovementStatus.size(); i++)
	{
		quadcopterMovementStatus[i] = CALCULATE_HOLD;
	}

	//usleep( 1000000 ); //FIXME usleep is no option, implement stabilize in land?

	/* Decline */
	 //Decline each quadcopter till it's not tracked anymore and then shutdown motor
	for(int i = 0; i < quadcopterMovementStatus.size(); i++)
	{
		quadcopterMovementStatus[i] = CALCULATE_LAND;
		
	}
	//TODO Do we need to reset after shutdown to make buildformation possible again? If yes how to differ from global shutdown?
	/*shutdownMutex.lock();
	this->landFinished = false;
	shutdownMutex.unlock();*/
 	ROS_INFO("Shutdown function finished");
	
}

/*
 * Shutdown
 * Set all values to zero and thrust to minimum. Send the values and exit program
 * after that.
 * 
 */
bool Controller::shutdown(control_application::Shutdown::Request  &req, control_application::Shutdown::Response &res)
{
	ROS_INFO("Service shutdown has been called");
	shutdownFormation ();
	void *resultCalc;
	pthread_join(tCalculateMovement, &resultCalc);
	void *resultBuild;
	pthread_join(tBuildFormation, &resultBuild);
	/* Unneccessary if move is not in loop */
	//void *resultSend;
	//pthread_join(tSend, &resultSend);
	ROS_INFO("Shutdown finished");
	return true;
}


/*
* Callback for Ros Subscriber of Formation Movement
*/
void Controller::MoveFormationCallback(const api_application::MoveFormation::ConstPtr &msg)
{
	ROS_INFO("I heard Movement. xMovement: %f", msg->xMovement);
	//float movement[3];
	std::vector<float> movement;
	movement.push_back( msg->xMovement );
	movement.push_back( msg->yMovement );
	movement.push_back( msg->zMovement );
	/*movement[0] = msg->xMovement;
	movement[1] = msg->yMovement;
	movement[2] = msg->zMovement;*/
	this->formationMovementMutex.lock();
	this->formationMovement.push_back(movement);
	this->formationMovementMutex.unlock();	
	this->lastFormationMovementMutex.lock();
	this->lastFormationMovement = getNanoTime();
	this->lastFormationMovementMutex.unlock();
	//calculate and set a new target position each time there is new data
	this->buildFormationMutex.lock();
	bool build = buildFormationFinished;
	this->buildFormationMutex.unlock();
	if(build)
	{
		setTargetPosition();
	}
}

/*
* Callback for Ros Subscriber of set Formation
*/
void Controller::SetFormationCallback(const api_application::SetFormation::ConstPtr &msg)
{
	ROS_INFO("I heard Formation. amount: %i", msg->amount);
	this->formation->setDistance(msg->distance);
	this->formation->setAmount(msg->amount);
	
	//TODO Delete when list arrays are converted to arrays list and target array size is used
	//this->amount = msg->amount;
	//Iterate over all needed quadcopters for formation and set the formation position of each quadcopter
	ROS_INFO("Setting Formation");
	Position6DOF formPos[msg->amount];
	for(int i = 0; i < msg->amount; i++)
	{
		double pos[3], ori[3];
		//double * pos;
		pos[0] = msg->xPositions[i];
		pos[1] = msg->yPositions[i];
		pos[2] = msg->zPositions[i];
		formPos[i].setPosition(pos);
		//Depends on the calculation of target and current position
		//ori[0] = 0;
		//ori[1] = 0;
		//ori[2] = 0;
		//formPos[i].setOrientation(ori);
	}
	this->formation->setPosition(formPos);
	ROS_INFO("Formation Position set");
	receivedFormMutex.lock();
	receivedFormation = true;
	receivedFormMutex.unlock();
	ROS_INFO("Set Formation done");
	return;
}

/*
 * Callback for Ros Subscriber of quadcopter status
 */
void Controller::QuadStatusCallback(const quadcopter_application::quadcopter_status::ConstPtr& msg, int topicNr)
{
	ROS_INFO("I heard Quadcopter Status. topicNr: %i", topicNr);
	//Intern mapping
	int quaId = this->getLocalId(topicNr);
	this->battery_status[quaId] = msg->battery_status;
	this->roll_stab[quaId] = msg->stabilizer_roll;
	this->pitch_stab[quaId] = msg->stabilizer_pitch;
	this->yaw_stab[quaId] = msg->stabilizer_yaw;
	this->thrust_stab[quaId] = msg->stabilizer_thrust;
	this->receivedQCStMutex.lock();
	this->receivedQuadStatus[quaId] = true;
	this->receivedQCStMutex.unlock();
}

/*
 * Callback for Ros Subscriber of system status. 1 = start, 2 = end
 */
void Controller::SystemCallback(const api_application::System::ConstPtr& msg)
{
	ROS_INFO("I heard System. Status: %i", msg->command);
	if(msg->command == 1)
	{
		initialize();
	}
	if(msg->command == 2)
	{
		shutdownMutex.lock();
		bool end = this->shutdownStarted;
		shutdownMutex.unlock();
		if(!end)
		{
			ROS_INFO("I want to shutdown");
			control_application::Shutdown srv;
			if(Shutdown_client.call(srv))
			{
				ROS_INFO("Shutdown call true");
			}
			//shutdown(NULL, NULL);
		}
		//TODO Do we need to clean up something here? Free space, join threads ...
	}
}


/* HELPER FUNCTIONS */
bool closeToTarget( Position6DOF position1, Position6DOF position2 )
{
	double distance = position1.getAbsoluteDistance( position2 );
	if( distance < RANGE_STABLE )
	{
		return true;
	}
	return false;
}

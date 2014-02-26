#include "Controller.hpp"

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
	this->BuildForm_srv  = this->n.advertiseService("BuildFormation", &Controller::buildFormation, this);
	this->Shutdown_srv = this->n.advertiseService("Shutdown", &Controller::shutdown, this);
	this->QuadID_srv = this->n.advertiseService("SetQuadcopters", &Controller::setQuadcopters, this);

	//Publisher
	//Publisher of Message to API
	this->Message_pub = this->n.advertise<api_application::Message>("Message", 100);
	

	//Client
	//this->FindAll_client = this->n.serviceClient<quadcopter_application::find_all>("find_all");
	this->Blink_client = this->n.serviceClient<quadcopter_application::blink>("blink");
	this->Announce_client = this->n.serviceClient<api_application::Announce>("Announce");
	this->Shutdown_client = this->n.serviceClient<control_application::Shutdown>("Shutdown");
	
	//All control variables are set to zero
	shutdownMutex.lock();
	this->shutdownStarted = 0;
	shutdownMutex.unlock();
	this->receivedQuadcopters = 0;
}

void* startThread(void* something)
{
	Controller *someOther = (Controller *) something; 
	someOther->calculateMovement();
}

void Controller::initialize()
{
	/*
	 * Start using threads here.
	 * tGet- One waiting to set and send new positions
	 * tCalc- One calculating the output
	 * 
	 * tGet is not a new thread, implemented as parent. 
	 * Leave function and wait to be called by position-instance.
	 */

	/* TODO: Error-handling. */
	pthread_create(&tCalc, NULL, startThread, this);

	shutdownMutex.lock();
	this->shutdownStarted = 0;
	shutdownMutex.unlock();
	api_application::Announce srv;
	srv.request.type = 2;
	//srv.request.camera_id = 0;
	if(Announce_client.call(srv))
	{
		this->senderID = srv.response.id;
	}
	//TODO What if service hasn't been called yet?
	if(receivedQuadcopters)
	{
		//Generate Subscribers and Publisher
		for(int i = 0; i < this->quadcopters.size(); i++)
		{
			//Subscriber to quadcopter status
			std::stringstream topicNameQS;
  			topicNameQS << "quadcopter_status_" << i;
			this->QuadStatus_sub[i] = this->n.subscribe<quadcopter_application::quadcopter_status>(topicNameQS.str().c_str(), 1000, boost::bind(&Controller::QuadStatusCallback, this, _1, i));

			//Publisher of Movement			
			std::stringstream topicNameMov;
  			topicNameMov << "quadcopter_movement_" << i;
			//Publisher for the Movement data of the Quadcopts (1000 is the max. buffered messages)
			this->Movement_pub[i] = this->n.advertise<control_application::quadcopter_movement>(topicNameMov.str().c_str(), 1000);
			
		}
	}
	getTracked = false;
}

void Controller::updatePositions(std::vector<Vector> positions, std::vector<int> ids, std::vector<int> updates)
{
		
	/* Save position vectors */	
	std::vector<Position6DOF> newListItem;
	int i = 0;
	int id;
	for(std::vector<Vector>::iterator it = positions.begin(); it != positions.end(); ++it, i++)
	{
		id = getLocalId(i);
		this->lastCurrent[id] = time(&this->lastCurrent[id]);
		Position6DOF newPosition = Position6DOF (it->getV1(), it->getV2(), it->getV3());
		newPosition.setTimestamp(this->lastCurrent[id]);
		newListItem.push_back( newPosition );
		if(this->quadcopterMovementStatus[id] == CALCULATE_START)
		{
		    this->quadcopterMovementStatus[id] = CALCULATE_STABILIZE;
		}
		tracked[id] = true;
		/*TODO: set quadcopterMovementStatus 
		 to CALCULATE_STABILIZE if it was CALCULATE_START before */
	}	
	std::size_t elements = positions.size();
	listPositionsMutex.lock();
	this->listPositions.push_back(newListItem);
	listPositionsMutex.unlock();
}

/*
void* startThreadMoveUp(void* something)
{
	Controller* other = (Controller*) something;
	other->moveUpNoArg();
}
*/

void Controller::reachTrackedArea(std::vector<int> ids)
{
	getTrackedMutex.lock();
	getTracked = true;
	getTrackedMutex.unlock();
	
	//TODO Use getLocalId???
	for(unsigned int i = 0; i < quadcopters.size(); i++)
	{
		for(unsigned int k = 0; k < ids.size(); k++)
		{
			if( quadcopters[i] == ids[k] )
			{
				quadcopterMovementStatus[i] = CALCULATE_START;
			}
		}
	}
	
}

void Controller::stopReachTrackedArea() 
{
	bool joinNecessary;

	getTrackedMutex.lock();
	joinNecessary = getTracked;
	getTracked = false;
	getTrackedMutex.unlock();

	for(unsigned int i = 0; i < quadcopterMovementStatus.size(); i++)
	{
		
		if( quadcopterMovementStatus[i] == CALCULATE_START )
		{
			//TODO We took Activated out of the header
			//quadcopterMovementStatus[i] = CALCULATE_ACTIVATED;
		}
	}
	
	if( joinNecessary )
	{
		/* TODO: Error-handling. */
		/*
		 void *resultGetTracked;
		pthread_join(tGetTracked, &resultGetTracked);
		*/
	}
}


	
/*
 * Calculates the movement vector of each quadcopter non stop and sends the vector first to convertMovement to set the yaw, pitch, roll and thrust
 * values. Then sends the movement to the quadcopter. Routine runs till shutdown formaiton is called.
 */	
void Controller::calculateMovement()
{
	bool inShutdown = false;
	shutdownMutex.lock();
	inShutdown = shutdownStarted;
	shutdownMutex.unlock();
	
	/* As long as we are not in the shutdown process, calculate new Movement data */
	while(!inShutdown)
	{
		checkInput();
		double moveVector[3];
		for(int i = 0; i < this->amount; i++)
		{	
			/* Shutdown */
			shutdownMutex.lock();
			inShutdown = shutdownStarted;
			shutdownMutex.unlock();
			if( inShutdown )
			{
				return;
			}

			/* Battery */
			if(this->battery_status[i] < LOW_BATTERY)
			{
				std::string message("Battery of Quadcopter %i is low (below %f). Shutdown formation\n", i, LOW_BATTERY);
				emergencyRoutine(message);
			}

			/* Calculation */
			tarPosMutex.lock();
			double * const target = this->listTargets.back()[i].getPosition();
			tarPosMutex.unlock();
			curPosMutex.lock();
			double * const current = this->listPositions.back()[i].getPosition();
			curPosMutex.unlock();

			switch( quadcopterMovementStatus[i] )
			{
				case CALCULATE_NONE:
					/* Take old values */
					moveVector[0] = INVALID;
					moveVector[1] = INVALID;
					moveVector[2] = INVALID;
					break;
				case CALCULATE_START:	
					/*TODO: adapt */
					//moveUp( i );
					break;
				case CALCULATE_STABILIZE:
					/*TODO*/
					stabilize( i );
					break;
				case CALCULATE_HOLD:					
					/*TODO hold and (if in shutdown) do it fast*/
					
					break;
				case CALCULATE_MOVE:
					moveVector[0] = target[0] - current[0];
					moveVector[1] = target[1] - current[1];
					moveVector[2] = target[2] - current[2];
					break;
				case CALCULATE_LAND:
					land( i );
					break;
				default:
					moveVector[0] = INVALID;
					moveVector[1] = INVALID;
					moveVector[2] = INVALID;
					break;
			}			
			/*
			 * Variation 1: convert movement and send movement
			 * Variation 2: convert movement, save movement and send all
			 */
			convertMovement(moveVector, i);
			/*TODO: Variation 2 */
			sendMovementAll();
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
void Controller::moveUp()
{
	/* Move Up All */
	for(unsigned int i = 0; i < quadcopterMovementStatus.size(); i++)
	{
		quadcopterMovementStatus[i] = CALCULATE_START;
	}
}

void Controller::moveUp(std::vector<int> ids)
{
	/* Move Up all mentioned */
	for(unsigned int i = 0; i < quadcopterMovementStatus.size(); i++)
	{
		for(unsigned int k = 0; k < ids.size(); k++)
		{
			if( quadcopters[i] == ids[i] )
			{
				quadcopterMovementStatus[i] = CALCULATE_START;
			}
		}			
	}
}

void Controller::moveUp( int internId )
{
	/* The actual calculation of "moving up" */
	bool continueMoveUp;
	double moveVector[3];	

	getTrackedMutex.lock();
	continueMoveUp = getTracked;
	getTrackedMutex.unlock();	
	if( continueMoveUp )
	{
		moveVector[0] = 0;
		moveVector[1] = 0;
		moveVector[2] = 100;
	} 
	else
	{
		moveVector[0] = INVALID;
		moveVector[1] = INVALID;
		moveVector[2] = INVALID;
	}
}


void Controller::stabilize( int internId )
{
	/* TODO */
}
void Controller::hold( int internId )
{
	/* TODO */
}

void Controller::land( int internId )
{
	//Decline until crazyflie isn't tracked anymore
	while(tracked[internId] == true)
	{
		this->movementAll[internId].setThrust(THRUST_DECLINE);
		sendMovementAll();
	}
	//Shutdown crazyflie after having left the tracking area.
	this->movementAll[internId].setThrust(THRUST_MIN);;
	sendMovementAll();
}

/*
 * Checks if formation movement data and quadcopter positions have been received lately. Otherwise calls emergencyroutine.
 */
bool Controller::checkInput()
{
	for(int i = 0; i < this->quadcopterMovementStatus.size(); i++)
	{
		time_t currentTime = time(&currentTime);
		if(currentTime - this->lastFormationMovement > TIME_UPDATED_END)
		{
		      std::string message("No new formation movement data has been received since %i sec. Shutdown formation\n", TIME_UPDATED_END);
		      emergencyRoutine(message);
		      return false;
		}
		if(currentTime - this->lastCurrent[i] > TIME_UPDATED_END)
		{
		      std::string message("No quadcopter position data has been received since %i sec. Shutdown formation\n", TIME_UPDATED_END);
		      emergencyRoutine(message);
		      tracked[i] = false;
		      return false;
		}
		if(currentTime - this->lastCurrent[i] > TIME_UPDATED_CRITICAL)
		{
		      tracked[i] = false;
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
	api_application::Message msg;
	msg.senderID = this->senderID;
	//Type 2 is a warning message
	msg.type = 2;
	msg.message = message;
	this->Message_pub.publish(msg);
	shutdownFormation();
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
	if( vector[0] == INVALID )
	{
		MovementQuadruple newMovement = MovementQuadruple(0, 0.0f, 0.0f, 0.0f);
		//TODO movementAll is not a list
		this->movementAll.push_back( newMovement );
	}
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
 * Creates a Ros message for the movement of the quadcopter and sends this 
 * to the quadcopter modul
 */
/*void Controller::sendMovement()
{
	//Creates a message for quadcopter Movement and sends it via Ros
	control_application::quadcopter_movement msg;
	msg.thrust = this->thrust;
	msg.roll = this->roll;
	msg.pitch = this->pitch;
	msg.yaw = this->yawrate;
	this->Movement_pub[id].publish(msg);	
}*/

/*
 * Creates a Ros message for the movement of each quadcopter and sends this 
 * to the quadcopter modul
 */
void Controller::sendMovementAll()
{
	//Creates a message for each quadcopter movement and sends it via Ros
	control_application::quadcopter_movement msg;
	for(int i = 0; i < movementAll.size(); i++)
	{
		msg.thrust = this->movementAll[i].getThrust();
		msg.roll = this->movementAll[i].getRoll();
		msg.pitch = this->movementAll[i].getPitch();
		msg.yaw = this->movementAll[i].getYawrate();
		this->Movement_pub[i].publish(msg);		/*FIXME while testing*/
	}
}

/*
 * Calculates the new Targets considering the previous targets and the formation movement vector (without orientation right now)
 */
void Controller::setTargetPosition()
{
	tarPosMutex.lock();
	std::vector<Position6DOF> latestTargets = this->listTargets.back();
	tarPosMutex.unlock();
	time_t currentTime = time(&currentTime);
	std::vector<Position6DOF> newTargets;
	//Iterate over all quadcopters in formation and set new target considering old target and formation Movement
	for(int i = 0; i < this->amount; i++)
	{
		tarPosMutex.lock();
		double * const targetOld = latestTargets[i].getPosition();
		tarPosMutex.unlock();
		double targetNew[3];
		formMovMutex.lock();
		targetNew[0] = targetOld[0] + this->formationMovement.back()[0];
		targetNew[1] = targetOld[1] + this->formationMovement.back()[1];
		targetNew[2] = targetOld[2] + this->formationMovement.back()[2];
		formMovMutex.unlock();
		//Check if new position would be in tracking area
		Vector vector = Vector(targetNew[0],targetNew[1],targetNew[2]);
		if(!this->trackingArea.contains(vector))
		{
			std::string message("Formation Movement is invalid. Quadcopter %i would leave Tracking Area.\n", i);
			emergencyRoutine(message);
			return;
		}
		newTargets[i].setPosition(targetNew);
		newTargets[i].setTimestamp(currentTime);
	}
	tarPosMutex.lock();
	this->listTargets.push_back(newTargets);
	tarPosMutex.unlock();
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
	for(int i = 0; i < req.amount; i++)
	{
		this->quadcopters[i] = req.quadcoptersId[i];
		this->quadcopterMovementStatus.push_back( CALCULATE_NONE );
		this->movementAll[i] = MovementQuadruple(0, 0, 0, 0);
		
	}
	receivedQuadcopters = true;
	return true;
}


/*
 * Builds Formation by starting one quadcopter after another, finding the right position and then
 * inclining a little to avoid collisions. So there is a "being tracked" and "moving" level, and a "standing still"
 * at the right position level.
 */
bool Controller::buildFormation(control_application::BuildFormation::Request  &req, control_application::BuildFormation::Response &res)
{
	//Get the formation Positions and the distance.
	Position6DOF* const formPos = this->formation->getPosition();
	double distance = this->formation->getDistance();
	//Pointer to the first tracked quadcopter
	double * first;
	std::vector<Position6DOF> newElement;
	this->listTargets.push_back(newElement);
	//Start one quadcopter after another
	for(int i = 0; i < this->amount; i++)
	{
		//Starting/ Inclining process
		quadcopterMovementStatus[i] = CALCULATE_START;
		//Calculate the wanted position for quadcopter i
		double * pos = formPos[i].getPosition();
		double target[3];
		target[0] = pos[0] * distance;
		target[1] = pos[1] * distance;
		target[2] = pos[2] * distance;
		//As long as the quadcopter isn't tracked, incline
		while(this->quadcopterMovementStatus[i] == CALCULATE_START)
		{
			//TODO When working right, do nothing here and just wait till it's tracked
			this->movementAll[i] = MovementQuadruple(THRUST_START, 0, 0, 0);
			sendMovementAll();
		}
		//If this is the first tracked quadcopter set it as a reference point for all the others
		if( i == 0)
		{
			curPosMutex.lock();
			first = listPositions.back()[0].getPosition();
			curPosMutex.unlock();
			tarPosMutex.lock();
			this->listTargets.back()[0].setPosition(first);
			tarPosMutex.unlock();
		}
		else
		{
			//Set all the other positions according to the first crazyflie
			target[0] += first[0];
			target[1] += first[1];
			target[2] += first[2];
			tarPosMutex.lock();
			this->listTargets.back()[i].setPosition(target);
			tarPosMutex.unlock();
			this->quadcopterMovementStatus[i] = CALCULATE_MOVE;
			//TODO When to switch back to HOLD?
		}
		//Incline a little bit to avoid collisions (there is a level with the qc which are already in position and a moving level)
		tarPosMutex.lock();
		target = this->listTargets.back()[i].getPosition();
		tarPosMutex.unlock();
		target[0] += 0;
		target[1] += distance;
		target[2] += 0;
		tarPosMutex.lock();
		this->listTargets.back()[i].setPosition(target);
		tarPosMutex.unlock();
		this->quadcopterMovementStatus[i] = CALCULATE_MOVE;
		//TODO When to switch back to HOLD?
	}
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
	/* Start shutdown process */
	shutdownMutex.lock();
	this->shutdownStarted = 1;
	shutdownMutex.unlock();
	
	/* Bring all quadcopters to a hold */	
	for(unsigned int i = 0; i < quadcopterMovementStatus.size(); i++)
	{
		quadcopterMovementStatus[i] = CALCULATE_HOLD;
	}

	/* Decline */
	 //Decline each quadcopter till it's not tracked anymore and then shutdown motor
	for(int i = 0; i < quadcopterMovementStatus.size(); i++)
	{
		quadcopterMovementStatus[i] = CALCULATE_LAND;
		
	}
	shutdownMutex.lock();
	this->shutdownStarted = 0;
	shutdownMutex.unlock();
	
}

/*
 * Shutdown
 * Set all values to zero and thrust to minimum. Send the values and exit program
 * after that.
 * 
 */
bool Controller::shutdown(control_application::Shutdown::Request  &req, control_application::Shutdown::Response &res)
{
	shutdownFormation ();
	void *resultCalc;
	pthread_join(tCalc, &resultCalc);
	
	/* Unneccessary if move is not in loop */
	//void *resultSend;
	//pthread_join(tSend, &resultSend);
	return true;
}


/*
* Callback for Ros Subscriber of Formation Movement
*/
void Controller::MoveFormationCallback(const api_application::MoveFormation::ConstPtr &msg)
{
	ROS_INFO("I heard: %f", msg->xMovement);
	//float movement[3];
	std::vector<float> movement;
	movement.push_back( msg->xMovement );
	movement.push_back( msg->yMovement );
	movement.push_back( msg->zMovement );
	/*movement[0] = msg->xMovement;
	movement[1] = msg->yMovement;
	movement[2] = msg->zMovement;*/
	formMovMutex.lock();
	this->formationMovement.push_back(movement);
	formMovMutex.unlock();	
	this->lastFormationMovement = time(&this->lastFormationMovement);
	//calculate and set a new target position each time there is new data
	setTargetPosition();
}

/*
* Callback for Ros Subscriber of set Formation
*/
void Controller::SetFormationCallback(const api_application::SetFormation::ConstPtr &msg)
{
	this->formation->setDistance(msg->distance);
	this->formation->setAmount(msg->amount);
	this->amount = msg->amount;
	//Iterate over all needed quadcopters for formation and set the formation position of each quadcopter
	Position6DOF formPos[this->amount];
	for(int i = 0; i < this->amount; i++)
	{
		double pos[3], ori[3];
		pos[0] = msg->xPositions[i];
		pos[1] = msg->yPositions[i];
		pos[2] = msg->zPositions[i];
		formPos[i].setPosition(pos);
		//Depends on the calculation of target and current position
		//ori[0] = msg->
		//ori[1] = msg->
		//ori[2] = msg->
		//formPos[i].setOrientation(ori);
	}
	this->formation->setPosition(formPos);
	//Initialize tracked (no quadcopter is tracked at the beginning)
	for(int i = 0; i < this->amount; i++)
	{
		this->tracked[i] = false;
	}
}

/*
 * Callback for Ros Subscriber of quadcopter status
 */
void Controller::QuadStatusCallback(const quadcopter_application::quadcopter_status::ConstPtr& msg, int topicNr)
{
	//Intern mapping
	int quaId = this->getLocalId(topicNr);
	this->battery_status[quaId] = msg->battery_status;
	this->roll_stab[quaId] = msg->stabilizer_roll;
	this->pitch_stab[quaId] = msg->stabilizer_pitch;
	this->yaw_stab[quaId] = msg->stabilizer_yaw;
	this->thrust_stab[quaId] = msg->stabilizer_thrust;
}

/*
 * Callback for Ros Subscriber of system status. 1 = start, 2 = end
 */
void Controller::SystemCallback(const api_application::System::ConstPtr& msg)
{
	if(msg->command == 1)
	{
		initialize();
	}
	if(msg->command == 2)
	{
		shutdownMutex.lock();
		bool inShutdown = shutdownStarted;
		shutdownMutex.unlock();
		if(!inShutdown)
		{
			control_application::Shutdown srv;
			Shutdown_client.call(srv);
			//shutdown(NULL, NULL);
		}
		//TODO Do we need to clean up something here? Free space, join threads ...
	}
}

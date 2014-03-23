#include "Controller.hpp"

bool closeToTarget( Position6DOF position1, Position6DOF position2 );
void* startThreadCalculateMovement(void* something);
void* startThreadBuildFormation(void* something);
void* startThreadShutdown(void* something);

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
	
	this->landMutex.lock();
	this->landFinished = false;
	this->landMutex.unlock();
	
	this->receivedQCMutex.lock();
	this->receivedQuadcopters = false; // received no quadcopters
	this->receivedQCMutex.unlock();
	
	
	this->buildFormationMutex.lock();
	this->buildFormationFinished = false; // has not built formation
	this->buildFormationMutex.unlock();
	
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
	this->time = getNanoTime();
	this->time2 = getNanoTime();
	this->time3 = getNanoTime();
	this->thrustTest = THRUST_START;
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

void Controller::setTrackingArea(TrackingArea area)
{
	this->trackingArea = area;
}

/*
 * Calculates the new Targets considering the previous targets and the formation movement vector (without orientation right now)
 */
void Controller::setTargetPosition()
{
	
	long int currentTime = getNanoTime();
	Position6DOF newTarget;	
	int formationAmount = this->formation->getAmount();
	//Iterate over all quadcopters in formation and set new target considering old target and formation Movement
	for(int i = 0; i < formationAmount; i++)
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
		/*if(!this->trackingArea.contains(vector))	// TODO carina
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
		while( this->listTargets[i].size() > 30 )
		{
			// Remove oldest elements
			this->listTargets[i].erase( this->listTargets[i].begin() );
		}
		this->listTargetsMutex.unlock();
	}	
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
		if(id == INVALID)
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
				this->movementStatusMutex.lock();
				if(this->quadcopterMovementStatus[id] == CALCULATE_START)
				{
					ROS_INFO("Stabilizing now %i", id);
					this->quadcopterMovementStatus[id] = CALCULATE_LAND;
				    this->shutdownMutex.lock();
					this->shutdownStarted = true;
					this->shutdownMutex.unlock();

				}
				this->movementStatusMutex.unlock();
			}
			//ROS_INFO("tracked id");
			trackedArrayMutex.lock();
			this->tracked[id] = true;
			trackedArrayMutex.unlock();
			this->listPositions[id].push_back( newPosition );
		} 
		//ROS_INFO("Push back");
		this->listPositionsMutex.lock(); 
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
 * Creates a Ros message for the movement of each quadcopter and sends this 
 * to the quadcopter modul
 */
void Controller::sendMovementAll()
{
	//Creates a message for each quadcopter movement and sends it via Ros
	control_application::quadcopter_movement msg;
	
	long int currentTime = getNanoTime();
	std::vector< MovementQuadruple > newListElement;
	for(int i = 0; i < listFutureMovement.size(); i++)
	{
		this->movementStatusMutex.lock();
		unsigned int quadStatus= this->quadcopterMovementStatus[i];
		this->movementStatusMutex.unlock();
		while( this->listFutureMovement[i].size() > 1 )
		{
			this->listFutureMovement[i].pop_back();
		}
		msg.thrust = this->listFutureMovement[i].front().getThrust();
		ROS_INFO("Trust of %i is %u",i, msg.thrust);
		if(quadStatus == CALCULATE_LAND || quadStatus == CALCULATE_START)
		{
			ROS_INFO("Send thrust movement all %u", msg.thrust);
		}
		msg.roll = this->listFutureMovement[i].front().getRoll();
		msg.pitch = this->listFutureMovement[i].front().getPitch();
		msg.yaw = this->listFutureMovement[i].front().getYawrate();
		this->Movement_pub[i].publish(msg);		
		//this->listFutureMovement[i].front().setTimestamp( currentTime );
		
		//FIXME Shouldn't we first erase and then add? Doesn't that erase newer items?
		// Save Element (TODO only if not too young)
		this->listSentQuadruples[i].push_back( this->listFutureMovement[i].front() );
		while( this->listSentQuadruples[i].size() > 5 )	// TODO global variable
		{
			// Remove oldest elements
			this->listSentQuadruples[i].erase( this->listSentQuadruples[i].begin() );
		}
	}
	//ROS_INFO("sendMovementAll finished");
}
	
/*
 * Calculates the movement vector of each quadcopter non stop and sends the vector first to convertMovement to set the yaw, pitch, roll and thrust
 * values. Then sends the movement to the quadcopter. Routine runs till shutdown formaiton is called.
 */	
void Controller::calculateMovement()
{
	ROS_INFO("Calculation started");
	//Keep track of the number of quadcopters landed
	int numberOfLanded = 0;
	this->movementStatusMutex.lock();
	int amount = quadcopterMovementStatus.size();
	this->movementStatusMutex.unlock();
	// As long as the land process isn't finished, we calculate new data
	bool end;
	this->receivedFormMutex.lock();
	if(this->receivedFormation)
	{
		/* When the number of quadcopters landed excesses the number of quadcopters in the formation, the land
		 * process is finished
		 */
		end = numberOfLanded >= this->formation->getAmount();
	}
	else
	{
		end = false;
	}
	this->receivedFormMutex.unlock();
	while(!end)
	{
		// Iterate over the total number of quadcopters as long as the land process isn't finished
		//ROS_INFO("Calculate");
		for(int i = 0; (i < amount) && (!end); i++)
		{	
			//Check if there is enough data to check input for correct data.
			receivedFormMutex.lock();
			receivedQCMutex.lock();
			bool enoughData = this->receivedFormation && this->receivedQuadcopters;
			receivedFormMutex.unlock();
			receivedQCMutex.unlock();
			this->movementStatusMutex.lock();
			unsigned int quadStatus = this->quadcopterMovementStatus[i];
			this->movementStatusMutex.unlock();
			if(quadStatus == CALCULATE_LAND || quadStatus == CALCULATE_HOLD) //only for testing TODO
			{
				//ROS_INFO("land or hold");
				if( enoughData)
				{
					checkInput(i);
				}
			}
			
			this->movementStatusMutex.lock();
			quadStatus= this->quadcopterMovementStatus[i];
			this->movementStatusMutex.unlock();
			switch( quadStatus )
			{
				case CALCULATE_NONE:
					if(numberOfLanded > 1)
					{
						 ROS_INFO("None: %i", i);
					}
					dontMove( i );
					break;
				case CALCULATE_START:	
					//ROS_INFO("Start %i", i);
					moveUp( i );
					break;
				case CALCULATE_STABILIZE:
					if( i == 0)
					{
						//ROS_INFO("Stabilize %i", i);
					}
					//stabilize( i );
					break;
				case CALCULATE_HOLD:	
					ROS_INFO("Hold %i", i);
					hold( i );				
					/* TODO */					
					break;
				case CALCULATE_MOVE:
					//ROS_INFO("Move %i", i);
					//convertMovement(i);
					break;
				case CALCULATE_LAND:
					if(numberOfLanded > 1 && i == 0)
					{
						 ROS_INFO("Landed: %i", numberOfLanded);
					}
					land( i, &numberOfLanded );
					break;
				default:
					break;
			}
			sendMovementAll();
			// Check if land process is finished and set control variable accordingly
			if(this->receivedFormation)
			{
				end = numberOfLanded >= this->formation->getAmount();
			}
			else
			{
				end = false;
			}
			landMutex.lock();
			this->landFinished = end;
			landMutex.unlock();
			
		}
	}	
}

/*
 * Builds Formation by starting one quadcopter after another, finding the right position and then
 * inclining a little to avoid collisions. So there is a "being tracked" and "moving" level, and a "standing still"
 * at the right position level.
 */
void Controller::buildFormation()
{
	ROS_INFO("Service buildFormation has been called");
	//Check if Formation and Quadcopters have been set and build formation can be started
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
	ROS_INFO("Formation Amount %i", formationAmount);
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
		ROS_INFO("Starting QC %i", i);
		//Starting/ Inclining process
		this->shutdownMutex.lock();
		bool shutdown = this->shutdownStarted;
		this->shutdownMutex.unlock();
		//If Shutdown has been called, abort. Otherwise start Starting process
		if(shutdown)
		{
			return;
		}
		else
		{
			this->movementStatusMutex.lock();
			this->quadcopterMovementStatus[i] = CALCULATE_START;
			this->time2 = getNanoTime();
			this->movementStatusMutex.unlock();
		}
		//Calculate Target Position of current qc using formation positions and the formation distance
		double pos[3];
		double target[3];
		for(int k = 0; k < 3; k++)
		{
			pos[k] = formPos[i].getPosition()[k];
			target[k] = pos[k] * distance;
		}
		this->movementStatusMutex.lock();
		unsigned int quadStatus = this->quadcopterMovementStatus[i];
		this->movementStatusMutex.unlock();
		//As long as the quadcopter isn't tracked, incline
		while(quadStatus == CALCULATE_START)
		{
        		this->movementStatusMutex.lock();
	        	quadStatus = this->quadcopterMovementStatus[i];
   		        this->movementStatusMutex.unlock();

			//ROS_INFO("Starting");
			//If Shutdown has been called, abort.
			this->shutdownMutex.lock();
			shutdown = this->shutdownStarted;
			this->shutdownMutex.unlock();
			if(shutdown)
			{
				ROS_INFO("Shutdown in BuildFormation");
				return;
			}
		}
		ROS_INFO("Tracked");
		//If this is the first tracked quadcopter set it as a reference point for all the others
		if( i == 0)
		{
			ROS_INFO("First one");
			this->listPositionsMutex.lock();
			//Get Position of first quadcopter
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
			//If Shutdown has been called, abort.
			this->shutdownMutex.lock();
			shutdown = this->shutdownStarted;
			this->shutdownMutex.unlock();
			if(shutdown)
			{
				return;
			}
			else
			{
				this->movementStatusMutex.lock();
				//FIXME for testing
				this->quadcopterMovementStatus[i] = CALCULATE_STABILIZE;
				//this->quadcopterMovementStatus[i] = CALCULATE_MOVE;
				this->movementStatusMutex.unlock();
			}
		}
		ROS_INFO("Inclining");
		//Incline a little bit to avoid collisions (there is a level with the qc which are already in position and a moving level)
		double pointer[3];
		this->listTargetsMutex.lock();
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
		this->listTargets[i].push_back(element);
		this->listTargetsMutex.unlock();
		this->shutdownMutex.lock();
		//If Shutdown has been called, abort.
		shutdown = this->shutdownStarted;
		this->shutdownMutex.unlock();
		if(shutdown)
		{
			return;
		}
		else
		{
			this->movementStatusMutex.lock();
			//FIXME for testing
			this->quadcopterMovementStatus[i] = CALCULATE_STABILIZE;
			//this->quadcopterMovementStatus[i] = CALCULATE_MOVE;
			this->movementStatusMutex.unlock();
		}
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
 * Service to set Quadcopter IDs
 */
bool Controller::setQuadcopters(control_application::SetQuadcopters::Request  &req, control_application::SetQuadcopters::Response &res)
{
	ROS_INFO("iService setQuadcopters has been called amount %i", req.amount);
	unsigned long int i;
	for( i = 0; i < req.amount; i++)
	{
		this->quadcopters.push_back(req.quadcoptersId[i]);
		this->movementStatusMutex.lock();
		this->quadcopterMovementStatus.push_back(CALCULATE_NONE);
		this->movementStatusMutex.unlock();
		
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
		MovementQuadruple noMovement = MovementQuadruple( 0, 0, 0, 0 );
		std::list<MovementQuadruple> newEmptyListMovement;	
		newEmptyListMovement.push_back( noMovement );
		this->listSentQuadruples.push_back(newEmptyListMovement);
		this->listFutureMovement.push_back(newEmptyListMovement);
		ROS_INFO("Initialization done");
		
		//Subscriber to quadcopter status
		std::stringstream topicNameQS;
		int id = this->quadcopters[i];
  		topicNameQS << "quadcopter_status_" << id;
		this->QuadStatus_sub[i] = this->n.subscribe<quadcopter_application::quadcopter_status>(topicNameQS.str().c_str(), 1000, boost::bind(&Controller::QuadStatusCallback, this, _1, i));
		
		//Publisher of Movement			
		std::stringstream topicNameMov;
  		topicNameMov << "quadcopter_movement_" << id;
		//Publisher for the Movement data of the Quadcopts (1000 is the max. buffered messages)
		this->Movement_pub[i] = this->n.advertise<control_application::quadcopter_movement>(topicNameMov.str().c_str(), 1000);
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
 * Shutdown
 * Set all values to zero and thrust to minimum. Send the values and exit program
 * after that.
 * 
 */
bool Controller::shutdown(control_application::Shutdown::Request  &req, control_application::Shutdown::Response &res)
{
	ROS_INFO("Service shutdown has been called");
	pthread_create(&tShutdownFormation, NULL, &startThreadShutdown, this);
	ROS_INFO("Thread tshutdownFormation set up");
	return true;
	
}

/*
 * Shutdown Formation
 */
void Controller::shutdownFormation()
{
	ROS_INFO("ShutdownFormation started");	
	shutdownMutex.lock();
	this->shutdownStarted = true; /* Start shutdown process */
	shutdownMutex.unlock();
	int formationAmount = this->formation->getAmount();
	
	/* Bring all quadcopters to a hold */
	for(unsigned int i = 0; i < formationAmount; i++)
	{
		this->movementStatusMutex.lock();
		quadcopterMovementStatus[i] = CALCULATE_HOLD;
		this->movementStatusMutex.unlock();
	}
	
	this->landMutex.lock();
	ROS_INFO("SHUTDOWN landFinished is %i", landFinished);
	bool end = this->landFinished;
	this->landMutex.unlock();
	while(!end)
	{
		this->landMutex.lock();
		end = this->landFinished;
		this->landMutex.unlock();
	}
	ROS_INFO("Join threads");
	void *resultCalc;
	pthread_join(tCalculateMovement, &resultCalc);
	void *resultBuild;
	pthread_join(tBuildFormation, &resultBuild);
	ROS_INFO("Shutdown function finished");	
}

int Controller::getLocalId(int globalId)	// TODO testme
{
  for(int i = 0; i < this->quadcopters.size() ; i++ )
  {
	if(globalId == this->quadcopters[i])
	{
	  return i;
	}
  }
  return INVALID;
}

/*
 * Checks if formation movement data and quadcopter positions have been received lately. Otherwise calls emergencyroutine.
 */
bool Controller::checkInput(int internId)
{
	ROS_INFO("Checking");
	this->receivedQCStMutex.lock();
	bool received = this->receivedQuadStatus[internId];
	this->receivedQCStMutex.unlock();
	this->movementStatusMutex.lock();
	unsigned int quadStatus= this->quadcopterMovementStatus[internId];
	this->movementStatusMutex.unlock();
	/* Battery */
	if(this->battery_status[internId] < LOW_BATTERY && quadStatus != CALCULATE_NONE && received)
	{
		std::string message("Battery of Quadcopter %i is low (below %f). Shutdown formation\n", internId, LOW_BATTERY);
		//ROS_INFO("Battery of Quadcopter %i is low (below %f). Shutdown formation\n", internId, LOW_BATTERY);
		//emergencyRoutine(message);
	}
	long int currentTime = getNanoTime();
	this->lastFormationMovementMutex.lock();
	long int lastForm = this->lastFormationMovement;
	this->lastFormationMovementMutex.unlock();
	if(currentTime - lastForm > TIME_UPDATED_END && quadStatus == CALCULATE_MOVE)
	{
		//std::string message = std::string("No new formation movement data has been received since %i sec. Shutdown formation\n", TIME_UPDATED_END);
		std::string message = "No new formation moevement data has been received";
		//ROS_INFO("No new formation movement data has been received since %i sec. Shutdown formation\n", TIME_UPDATED_END);
		//emergencyRoutine(message);
		return false;
	}
	this->lastCurrentMutex.lock();
	long int lastCur = this->lastCurrent[internId];
	this->lastCurrentMutex.unlock();
	if(currentTime - lastCur > TIME_UPDATED_END && quadStatus != CALCULATE_NONE && quadStatus != CALCULATE_START)
	{
		//ROS_INFO("No quadcopter position data has been received since %i sec. Shutdown formation\n", TIME_UPDATED_END);
		//std::string message2 = std::string("No quadcopter position data has been received since %i sec. Shutdown formation\n", TIME_UPDATED_END);
		std::string message2 = "No new quadcopter position data has been received";
		//emergencyRoutine(message2);
		trackedArrayMutex.lock();
		ROS_INFO("tracked false");
		tracked[internId] = false;
		trackedArrayMutex.unlock();
		return false;
	}
	//ROS_INFO("Critical");
	if(currentTime - lastCur > TIME_UPDATED_CRITICAL && quadStatus != CALCULATE_NONE && quadStatus != CALCULATE_START)
	{
		trackedArrayMutex.lock();
		ROS_INFO("tracked false");	
		tracked[internId] = false;
		trackedArrayMutex.unlock();
		return false;
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
		if(Shutdown_client.call(srv))
		{
			ROS_INFO("Shutdown call true");
		}
	}
}

/*
* Callback for Ros Subscriber of Formation Movement
*/
void Controller::MoveFormationCallback(const api_application::MoveFormation::ConstPtr &msg)
{
	ROS_INFO("I heard Movement. xMovement: %f", msg->xMovement);
	std::vector<float> movement;
	movement.push_back( msg->xMovement );
	movement.push_back( msg->yMovement );
	movement.push_back( msg->zMovement );
	this->formationMovementMutex.lock();
	this->formationMovement.push_back(movement);
	this->formationMovementMutex.unlock();	
	this->lastFormationMovementMutex.lock();
	this->lastFormationMovement = getNanoTime();
	this->lastFormationMovementMutex.unlock();
	//calculate and set a new target position each time there is new data
	this->buildFormationMutex.lock();
	bool build = this->buildFormationFinished;
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
	//Iterate over all needed quadcopters for formation and set the formation position of each quadcopter
	Position6DOF formPos[msg->amount];
	for(int i = 0; i < msg->amount; i++)
	{
		double pos[3];
		pos[0] = msg->xPositions[i];
		pos[1] = msg->yPositions[i];
		pos[2] = msg->zPositions[i];
		formPos[i].setPosition(pos);
	}
	this->formation->setPosition(formPos);
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
	//ROS_INFO("I heard Quadcopter Status. topicNr: %i", topicNr);
	//Intern mapping
	int quaId = this->getLocalId(topicNr);
	this->battery_status[quaId] = msg->battery_status;
	this->roll_stab[quaId] = msg->stabilizer_roll;
	this->pitch_stab[quaId] = msg->stabilizer_pitch;
	this->yaw_stab[quaId] = msg->stabilizer_yaw;
	this->thrust_stab[quaId] = msg->stabilizer_thrust;
	long int currentTime = getNanoTime();
	if(quaId == 0 && currentTime > this->time + 2000000000)
	{
		ROS_INFO("bat: %f, roll: %f, pitch: %f, yaw: %f, thrust: %u", msg->battery_status, msg->stabilizer_roll, msg->stabilizer_pitch, msg->stabilizer_yaw, msg->stabilizer_thrust);
		this->time = currentTime;
	}
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
			/*if(Shutdown_client.call(srv))
			{
				ROS_INFO("Shutdown call true");
			}*/
			shutdown(srv.request, srv.response);
		}
		//TODO Do we need to clean up something here? Free space, join threads ...
	}
}

void Controller::dontMove( int internId)
{
	MovementQuadruple newMovement = MovementQuadruple( 0, 0, 0, 0 );
	this->listFutureMovement[internId].clear();
	long int currentTime = getNanoTime();
	newMovement.setTimestamp( currentTime );
	this->listFutureMovement[internId].push_front( newMovement );
	
}

void Controller::moveUp( int internId )
{
	bool moveUpSmart = false;
	//TODO Why don't we set a timestamp for newMovement?
	if( !moveUpSmart ) {		
		MovementQuadruple newMovement = MovementQuadruple( this->thrustTest, 0, 0, 0 );
		ROS_DEBUG("Thrust is %u", this->thrustTest);
		this->listFutureMovement[internId].clear();
		this->listFutureMovement[internId].push_front( newMovement );
		long int current = getNanoTime();
		//Increases thrust step by step to ensure slow inclining
		if(current > this->time3 + 10000000)
		{
			usleep(85000);
			this->thrustTest += 700;
			this->time3 = getNanoTime();
		}
		//Protection mechanism for qc (either a too high thrust value or start process took too long)
		if(this->thrustTest >= 50000 || current > this->time2 + 4000000000)
		{
			ROS_DEBUG("Emergency Shutdown Test");
			this->shutdownMutex.lock();
			this->shutdownStarted = true;
			this->shutdownMutex.unlock();
			this->movementStatusMutex.lock();	
			quadcopterMovementStatus[internId] = CALCULATE_LAND;
			this->movementStatusMutex.unlock();
		}
			
		/*ROS_INFO("Send Movement here for testing in moveup");
        	control_application::quadcopter_movement msg;
        	msg.thrust = this->listFutureMovement[internId].back().getThrust();
        	ROS_INFO("Thrust in land is %u", msg.thrust);
        	msg.roll = this->listFutureMovement[internId].back().getRoll();
       		msg.pitch = this->listFutureMovement[internId].back().getPitch();
       		msg.yaw = this->listFutureMovement[internId].back().getYawrate();
        	this->Movement_pub[internId].publish(msg);
		/*msg.thrust = 12005;
                ROS_INFO("Thrust in land is %i", msg.thrust);
                msg.roll = 0;
                msg.pitch = 0;
                msg.yaw = 0;
                this->Movement_pub[internId].publish(msg);*/		
	} else
	{
		if( this->listFutureMovement[internId].size() == 0 )
		{
			int diff = 80;
			long int timeDiff = 400000000;
			long int currentTime = getNanoTime();
			MovementQuadruple newMovement = MovementQuadruple( THRUST_START, 0, 0, 0, currentTime );
			this->listFutureMovement[internId].push_back( newMovement );
		
			newMovement.setTimestamp( newMovement.getTimestamp() + timeDiff );
			newMovement.setThrust( newMovement.getThrust() + diff );
			this->listFutureMovement[internId].push_back( newMovement );
		
			newMovement.setTimestamp( newMovement.getTimestamp() + timeDiff );
			newMovement.setThrust( newMovement.getThrust() + diff );
			this->listFutureMovement[internId].push_back( newMovement );
		}
	}
}


void Controller::stabilize( int internId )
{
	Interpolator interpolator = Interpolator();
	this->listTargetsMutex.lock();
	Position6DOF targetInternId = this->listTargets[internId].back();
	this->listTargetsMutex.unlock();
	/*
	if() // TODO dateNow-dateOfLatestCalculation < CALCULATE_STABILIZE_STEP )
	{
		return;
	}
	*/
	MovementQuadruple newMovement = interpolator.calculateNextMQ(this->listSentQuadruples[internId], this->listPositions[internId], targetInternId, internId);
	
	this->listFutureMovement[internId].clear();
	this->listFutureMovement[internId].push_front( newMovement );
	   
}

void Controller::hold( int internId )
{
	/* TODO */
	this->movementStatusMutex.lock();
	ROS_INFO("%i now land", internId);
	quadcopterMovementStatus[internId] = CALCULATE_LAND;
	this->movementStatusMutex.unlock();		
	
}

void Controller::land( int internId, int * nrLand )
{
	ROS_INFO("Land");
	long int currentTime = getNanoTime();
	this->trackedArrayMutex.lock();
	//Decline until crazyflie isn't tracked anymore
	if(tracked[internId] == true)
	{
		ROS_INFO("Declining ros");
		/*while( this->listFutureMovement[internId].size() > 1)	// FIXME @dominik
		{
			this->listFutureMovement[internId].pop_back();
		}*/
		//MovementQuadruple newMovement = MovementQuadruple( THRUST_DECLINE, 0, 0, 0 ); TODO @problem sendMovementAll
		MovementQuadruple newMovement = this->listFutureMovement[internId].front();
		newMovement.setThrust( THRUST_DECLINE );
		newMovement.setTimestamp( currentTime );
		this->listFutureMovement[internId].push_front( newMovement );		
	}
	else
	{
		ROS_INFO("min");
		//Shutdown crazyflie after having left the tracking area.
		//MovementQuadruple newMovement = MovementQuadruple( THRUST_DECLINE, 0, 0, 0 ); TODO @problem sendMovementAll
		MovementQuadruple newMovement = this->listFutureMovement[internId].front();
		newMovement.setThrust( THRUST_MIN );
		newMovement.setTimestamp(currentTime);
		this->listFutureMovement[internId].push_front( newMovement );
		this->movementStatusMutex.lock();
		this->quadcopterMovementStatus[internId] = CALCULATE_NONE;
		this->movementStatusMutex.unlock();
		(*nrLand)++;
		ROS_INFO("Landed: %i", *nrLand);
	}
	this->trackedArrayMutex.unlock();
	//TODO This is here just for testing.
	ROS_INFO("Send Movement here for testing");
	control_application::quadcopter_movement msg;
	msg.thrust = this->listFutureMovement[internId].front().getThrust();
	ROS_INFO("Thrust in land is %u", msg.thrust);
	msg.roll = this->listFutureMovement[internId].front().getRoll();
	msg.pitch = this->listFutureMovement[internId].front().getPitch();
	msg.yaw = this->listFutureMovement[internId].front().getYawrate();
	this->Movement_pub[internId].publish(msg);	
}


/* HELPER FUNCTIONS */
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
			if( counter==compareTime[0] || counter==compareTime[1] || counter==compareTime[2] )
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

bool closeToTarget( Position6DOF position1, Position6DOF position2 )
{
	double distance = position1.getAbsoluteDistance( position2 );
	if( distance < RANGE_STABLE )
	{
		return true;
	}
	return false;
}

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

void* startThreadShutdown(void* something)
{
	Controller *someOther = (Controller *) something;
	someOther->shutdownFormation();
}


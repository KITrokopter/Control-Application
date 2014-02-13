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
	//TODO multiple topics
	//Subscriber for the quadcopter status data of the Quadcopters (1000 is the max. buffered messages)
	//this->QuadStatus_sub = this->n.subscribe("quadcopter_status", 1000, &Controller::QuadStatusCallback, this);

	//Service
	//Service for BuildFormation and Shutdown
	this->BuildForm_srv  = this->n.advertiseService("BuildFormation", &Controller::buildFormation, this);
	this->Shutdown_srv = this->n.advertiseService("Shutdown", &Controller::shutdown, this);

	/*
	//Publisher
	//Publisher for the Movement data of the Quadcopts (1000 is the max. buffered messages)
	this->Movement_pub = this->n.advertise<control_application::quadcopter_movement>("quadcopter_movement", 1000);
	*/

	//Client
	this->FindAll_client = this->n.serviceClient<quadcopter_application::find_all>("find_all");
	this->Blink_client = this->n.serviceClient<quadcopter_application::blink>("blink");
	
	//Initializes invalid currentPosition and targetPosition
	double invalid[3] = {INVALID, INVALID, INVALID};
	this->currentPosition[0].setPosition(invalid);
	this->targetPosition[0].setPosition(invalid);
	//All control variables are set to zero
	this->shutdownStarted = 0;
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
	std:pthread_create(&tCalc, NULL, &Controller::calculateMovement, NULL);

	shutdownMutex.lock();
	this->shutdownStarted = 0;
	shutdownMutex.unlock();
	
	//Initialization of quadcopters (find_all and create mapping + subscribe)
	quadcopter_application::find_all srv;
	//TODO initialization
	/*srv.request.seq = (unsigned int)0;
	srv.request.stamp = (time)0;
	srv.request.frame_id = "";*/
	std::string * uri;
	if(FindAll_client.call(srv))
	{
		//TODO Get amount of quadcopter over api instead
		//uri = srv.response.uri;
		//this->totalAmount = srv.response.amount;
		
		//TODO Create map uri->id
		//Generate Subscribers and Publisher
		for(int i = 0; i < this->totalAmount; i++)
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
	time_t currentTime = time(&currentTime);
	for(std::vector<Vector>::iterator it = positions.begin(); it != positions.end(); ++it)
	{
		Position6DOF newPosition = Position6DOF (*it.getV1(), *it.getV2(), *it.getV3());
		newListItem.push_back( newPosition );		
	}	
	std::size_t elements = positions.size();
	listPositionsMutex.lock();
	this->listPositions.push_back(newListItem);
	listPositionsMutex.unlock();
}

void Controller::reachTrackedArea(std::vector<int> ids)
{
	getTrackedMutex.lock();
	getTracked = true;
	getTrackedMutex.unlock();
	
	/* TODO: Error-handling. */
	/* Test ... if enough time: fix it, if not: copy to private variable */
	std:pthread_create(&tGetTracked, NULL, &Controller::moveUp, ids); 	
}

void Controller::stopReachTrackedArea() 
{
	bool joinNecessary;

	getTrackedMutex.lock();
	joinNecessary = getTracked;
	getTracked = false;
	getTrackedMutex.unlock();

	if( joinNecessary )
	{
		/* TODO: Error-handling. */
		void *resultGetTracked;
		pthread_join(tGetTracked, &resultGetTracked);
	}
}

void Controller::moveUp(std::vector<int> ids)
{
	bool continueMoveUp = true;
	while(continueMoveUp)
	{
		double moveVector[] = {0, 0, 100};
		for(int i = 0; i < ids.size(); i++)
		{		
			this->id = ids[i];
			//Convert Movement vector to thrust, pitch... data
			convertMovement(moveVector);
			//Send Movement to the quadcopter
			sendMovement();
		}
		// usleep(1000); // microseconds
		getTrackedMutex.lock();
		continueMoveUp = getTracked;
		getTrackedMutex.unlock();
	}
}
		
void Controller::calculateMovement()
{
	
	/* As long as we are not in the shutdown process, calculate new Movement data */
	while(!shutdownStarted)
	{
		double moveVector[3];
		for(int i = 0; i < this->amount; i++)
		{		
			//Gets the right hardware id/ String id
			//this->idString = this->quadcopters[i];
			this->id = i;
			tarPosMutex.lock();
			double * const target = this->targetPosition[i].getPosition();
			tarPosMutex.unlock();
			curPosMutex.lock();
			double * const current = this->currentPosition[i].getPosition();
			curPosMutex.unlock();
			moveVector[0] = target[0] - current[0];
			moveVector[1] = target[1] - current[1];
			moveVector[2] = target[2] - current[2];
			//Convert Movement vector to thrust, pitch... data
			convertMovement(moveVector);
			//Send Movement to the quadcopter
			sendMovement();
		}
		/*while(this->newTarget == 0 && this->newCurrent == 0) 
		{
			// delete if compiling fails
			usleep(1000); // microseconds
		}*/
	}	
}

void Controller::sendMovement()
{
	//Creates a message for quadcopter Movement and sends it via Ros
	control_application::quadcopter_movement msg;
	msg.thrust = this->thrust;
	msg.yaw = this->yawrate;
	msg.pitch = this->pitch;
	msg.roll = this->roll;
	this->Movement_pub[id].publish(msg);	
}

void Controller::convertMovement(double* vector)
{
	/* conversion from vectors to thrust, yawrate, pitch... */
	int thrust_react_z_low = -5;
	int thrust_react_z_high = 5;
	
	if (vector[2] > thrust_react_z_high) {
		this->thrust += THRUST_STEP;
	} else if (vector[2] < thrust_react_z_high) {
		this->thrust -= THRUST_STEP;
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

	this->roll = ratio_roll + ROLL_STEP;
	this->pitch = ratio_pitch + PITCH_STEP;

	this->yawrate = 0.0;
}

//Move the last Positions according to the formation movement vector (without orientation right now)
void Controller::setTargetPosition()
{
	//Iterate over all quadcopters in formation and set new target considering old target and formation Movement
	for(int i = 0; i < this->amount; i++)
	{
		tarPosMutex.lock();
		double * const targetOld = this->targetPosition[i].getPosition();
		tarPosMutex.unlock();
		double targetNew[3];
		targetNew[0] = targetOld[0] + this->formationMovement[0];
		targetNew[1] = targetOld[1] + this->formationMovement[1];
		targetNew[2] = targetOld[2] + this->formationMovement[2];
		this->targetPosition[i].setPosition(targetNew);
	}
}


/*
 * Builds Formation by starting one quadcopter after another, finding the right position and then
 * inclining a little to avoid collisions. So there is a "being tracked" and "moving" level, and a "standing still"
 * at the right position level.
 */
bool Controller::buildFormation(control_application::BuildFormation::Request  &req, control_application::BuildFormation::Response &res)
{
	//Get the formation Positions and the distance.
	Position6DOF* const formPos = this->formation.getPosition();
	double distance = this->formation.getDistance();
	//Pointer to the first tracked quadcopter
	double * first;
	//Start one quadcopter after another
	for(int i = 0; i < this->amount; i++)
	{
		//Starting/ Inclining process
		//this->idString = this->quadcopters[i];
		this->id = i;
		this->thrust = THRUST_START;
		this->yawrate = 0;
		this->pitch = 0;
		this->roll = 0;
		//Calculate the wanted position for quadcopter i
		double * pos = formPos[i].getPosition();
		double target[3];
		target[0] = pos[0] * distance;
		target[1] = pos[1] * distance;
		target[2] = pos[2] * distance;
		//Set target Position for quadcopter i
		this->targetPosition[i].setPosition(target);
		//As long as the quadcopter isn't tracked, incline
		while(!this->tracked[i])
		{
			sendMovement();
		}
		//If this is the first tracked quadcopter set it as a reference point for all the others
		if( i == 0)
		{
			curPosMutex.lock();
			first = this->currentPosition[0].getPosition();
			curPosMutex.unlock();
		}
		else
		{
			//Set all the other positions according to the first crazyflie
			target[0] += first[0];
			target[1] += first[1];
			target[2] += first[2];
			tarPosMutex.lock();
			this->targetPosition[i].setPosition(target);
			tarPosMutex.unlock();
			//Calculate Movement to the wanted position + convert it + send it
			calculateMovement();
		}
		//Incline a little bit to avoid collisions (there is a level with the qc which are already in position and a moving level)
		target[0] = 0;
		target[1] = distance;
		target[2] = 0;
		tarPosMutex.lock();
		this->targetPosition[i].setPosition(target);
		tarPosMutex.unlock();
		calculateMovement();
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
	//Shutdown process is started
	this->shutdownStarted = 1;
	//Bring all quadcopters to a stand
	for(int i = 0; i < this->amount; i++)
	{
		//this->idString = this->quadcopters[i];
		this->id = i;
		this->thrust = THRUST_STAND_STILL;
		this->yawrate = 0;
		this->pitch = 0;
		this->roll = 0;
		this->shutdownStarted = 0;
		sendMovement();
	}
	//Decline each quadcopter till it's not tracked anymore and then shutdown motor
	for(int i = 0; i < this->amount; i++)
	{
		this->id = i;
		curPosMutex.lock();
		double * const current = this->currentPosition[id].getPosition();
		curPosMutex.unlock();
		//Decline until crazyflie isn't tracked anymore
		while(tracked[i] != INVALID)
		{
			this->thrust = THRUST_DECLINE;
			sendMovement();
		}
		//Shutdown crazyflie after having left the tracking area.
		this->thrust = THRUST_MIN;
		sendMovement();
	}
	//Shutdown process is finished
	this->shutdownStarted = 0;
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
	this->formationMovement[0] = msg->xMovement;
	this->formationMovement[1] = msg->yMovement;
	this->formationMovement[2] = msg->zMovement;
	//calculate and set a new target position each time there is new data
	setTargetPosition();
}

/*
* Callback for Ros Subscriber of set Formation
*/
void Controller::SetFormationCallback(const api_application::SetFormation::ConstPtr &msg)
{
	this->formation.setDistance(msg->distance);
	this->formation.setAmount(msg->amount);
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
	this->formation.setPosition(formPos);
	//Initialize tracked (no quadcopter is tracked at the beginning)
	for(int i = 0; i < this->amount; i++)
	{
		this->tracked[i] = 0;
	}
}

/*
* Callback for Ros Subscriber of quadcopter status
*/
void Controller::QuadStatusCallback(const quadcopter_application::quadcopter_status::ConstPtr& msg, int topicNr)
{
	this->battery_status[topicNr] = msg->battery_status;
	this->roll_stab[topicNr] = msg->stabilizer_roll;
	this->pitch_stab[topicNr] = msg->stabilizer_pitch;
	this->yaw_stab[topicNr] = msg->stabilizer_yaw;
	this->thrust_stab[topicNr] = msg->stabilizer_thrust;
}

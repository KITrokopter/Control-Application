#include "Controller.hpp"

Controller::Controller(std::vector<Position6DOF> targetPosition, std::vector<Position6DOF> currentPosition, Formation formation)
{
	/**
  	 * NodeHandle is the main access point to communications with the ROS system.
	 * The first NodeHandle constructed will fully initialize this node, and the last
     * NodeHandle destructed will close down the node.
     */
    ros::NodeHandle n;
	//Subscriber for the MoveFormation data of the Quadcopts (1000 is the max. buffered messages)
	this->MoveFormation_sub = n.subscribe("MoveFormation", 1000, &Controller::MoveFormationCallback, this);
	this->SetFormation_sub = n.subscribe("SetFormation", 100, &Controller::SetFormationCallback, this);

	//Publisher for the Movement data of the Quadcopts (1000 is the max. buffered messages)
	this->Movement_pub = n.advertise<control_application::Movement>("Movement", 1000);
	
	this->formation.setDistance(formation.getDistance());
	this->formation.setAmount(formation.getAmount());
	this->formation.setPosition(formation.getPosition());
	for(int i = 0; i < amount; i++) 
	{
		this->targetPosition[i] = targetPosition[i];
		this->currentPosition[i] = currentPosition[i];
	}
}

Controller::Controller()
{
	/**
  	 * NodeHandle is the main access point to communications with the ROS system.
	 * The first NodeHandle constructed will fully initialize this node, and the last
     * NodeHandle destructed will close down the node.
     */
    ros::NodeHandle n;
	//Subscriber for the MoveFormation data of the Quadcopts (1000 is the max. buffered messages)
	this->MoveFormation_sub = n.subscribe("MoveFormation", 1000, &Controller::MoveFormationCallback, this);
	this->SetFormation_sub = n.subscribe("SetFormation", 100, &Controller::SetFormationCallback, this);

	//Publisher for the Movement data of the Quadcopts (1000 is the max. buffered messages)
	this->Movement_pub = n.advertise<control_application::Movement>("Movement", 1000);
	double invalid[3] = {INVALID, INVALID, INVALID};
	this->currentPosition[0].setPosition(invalid);
	this->targetPosition[0].setPosition(invalid);
	this->formation.setAmount(INVALID);
	this->newTarget = 0;
	this->shutdown = 0;
}

void Controller::initialize()
{
	/*
	 * Start using threads here.
	 * tGet- One waiting to set new positions
	 * tCalc- One calculating the output
	 * tSend- One sending movement-data over ROS
	 * 
	 * TODO: Could the following work?
	 * tGet is not a new thread, implemented as parent. 
	 * Leave function and wait to be called by position-instance?
	 */

	/* TODO: Error-handling. */
	std:pthread_create(&tCalc, NULL, &calculateMovement, NULL);
	std:pthread_create(&tSend, NULL, &move, NULL);
}
		
void Controller::calculateMovement()
{

	/* TODO:  */
	
	/* TODO: pthread, while shutdown=no do run the infinte loop */
	while(!shutdown)
	{
		double moveVector[3];
		for(int i = 0; i < amount; i++)
		{		
			this->idString = this->quadcopters[i];
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
			convertMovement(moveVector);
			move();
		}
	}	
}

void Controller::move()
{
	/* TODO: pthread, while shutdown=no do run the infinte loop */
	while(!shutdown)
	{	
		control_application::Movement msg;
		tarPosMutex.lock();
		double * const target = this->targetPosition[id].getPosition();
		tarPosMutex.unlock();
		curPosMutex.lock();
		double * const current = this->currentPosition[id].getPosition();
		curPosMutex.unlock();
		this->newTarget = 0;
		//msg.id = this->idString;
		msg.id = this->id;
		msg.thrust = this->thrust;
		msg.yaw = this->yawrate;
		msg.pitch = this->pitch;
		msg.roll = this->roll;

		// Send values until targed is reached.
		//TODO: change it
		while(current[0] == INVALID || POS_CHECK)	 //Either the current position is invalid because qc not tracked yet or we try to reach the target position
		{
			this->Movement_pub.publish(msg);
			//If a new target is set, the newTarget variable is true and we start a new calculation	
			if(newTarget)
			{
				return;
			}	
		}
		if(startProcess)
		{
			msg.thrust = THRUST_STAND_STILL;
			this->Movement_pub.publish(msg);
		}
	}
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
		/* Not sure what to do here, maybe nothing. */
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
	for(int i = 0; i < amount; i++)
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
	this->newTarget = 1;
}


/*
 * Gehe davon aus dass es ein Topic mit Quadcopters gibt mit URI/Hardware ID 
 * und diese dem quadcopter array zugewiesen wurde qc[id][uri/hardware id]
 */
void Controller::buildFormation()
{
	Position6DOF* const formPos = this->formation.getPosition();
	double distance = this->formation.getDistance();
	double * first;
	for(int i = 0; i < amount; i++)
	{
		this->idString = this->quadcopters[i];
		this->id = i;
		this->thrust = THRUST_START;
		this->yawrate = 0;
		this->pitch = 0;
		this->roll = 0;
		double * pos = formPos[i].getPosition();
		double target[3];
		target[0] = pos[0] * distance;
		target[1] = pos[1] * distance;
		target[2] = pos[2] * distance;
		this->targetPosition[i].setPosition(target);
		move();	
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
}

void Controller::shutdownFormation()
{
	this->shutdown = 1;
	//Bring all quadcopters to a stand
	for(int i = 0; i < amount; i++)
	{
		this->idString = this->quadcopters[i];
		this->id = i;
		this->thrust = THRUST_STAND_STILL;
		this->yawrate = 0;
		this->pitch = 0;
		this->roll = 0;
		this->shutdown = 0;
		move();
	}
	//Decline each quadcopter till it's not tracked anymore and then shutdown motor
	for(int i = 0; i < amount; i++)
	{
		curPosMutex.lock();
		double * const current = this->currentPosition[id].getPosition();
		curPosMutex.unlock();
		while(current[0] != INVALID)
		{
			//TODO Check for collisions when declining
			this->thrust = THRUST_DECLINE;
			move();
		}
		//TODO Is this point to high?
		this->thrust = THRUST_MIN;
		move();
	}
	this->shutdown = 1;
}

void Controller::shutdown()
{
	shutdownFormation ();

	void *resultCalc, *resultSend;
	pthread_join(tCalc, &resultCalc);
	pthread_join(tSend, &resultSend);
}

void Controller::MoveFormationCallback(const control_application::MoveFormation::ConstPtr &msg)
{
	ROS_INFO("I heard: %f", msg->xMovement);
	this->formationMovement[0] = msg->xMovement;
	this->formationMovement[1] = msg->yMovement;
	this->formationMovement[2] = msg->zMovement;
}

void Controller::SetFormationCallback(const api_application::SetFormation::ConstPtr &msg)
{
	this->formation.setDistance(msg->distance);
	this->formation.setAmount(msg->amount);
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
}

#include "Controller.hpp"

Controller::Controller(Position6DOF targetPosition[], Position6DOF currentPosition[], Formation formation)
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
	int invalid[3] = {INVALID, INVALID, INVALID};
	this->currentPosition[0].setPosition(invalid);
	this->targetPosition[0].setPosition(invalid);
	this->formation.setAmount(INVALID);
}

void Controller::initialize()
{
	
}
		
void Controller::calculateMovement()
{
	int moveVector[3];
	for(int i = 0; i < amount; i++)
	{		
		this->idString = this->quadcopters[i];
		this->id = i;
		int * target = this->targetPosition[i].getPosition();
		int * current = this->currentPosition[i].getPosition();
		moveVector[0] = target[0] - current[0];
		moveVector[1] = target[1] - current[1];
		moveVector[2] = target[2] - current[2];
		convertMovement(moveVector);
		move();
	}	
}

void Controller::move()
{
	control_application::Movement msg;
	int * check = this->currentPosition[id].getPosition();
	int * target = this->targetPosition[id].getPosition();
	//msg.id = this->idString;
	msg.id = this->id;
	msg.thrust = this->thrust;
	msg.yaw = this->yawrate;
	msg.pitch = this->pitch;
	msg.roll = this->roll;

	// Send values until targed is reached.
	//TODO: change it
	while(check[0] != INVALID || POS_CHECK)		//TODO: changed by Do.
	{
		this->Movement_pub.publish(msg);
		check = this->currentPosition[id].getPosition();		
	}
	if(startProcess)
	{
		msg.thrust = STAND_STILL;
		this->Movement_pub.publish(msg);
	}
}

void Controller::convertMovement(int* vector)
{
	/* conversion from vectors to thrust, yawrate, pitch... */
	int thrust_react_z_low = -5;
	int thrust_react_z_high = 5;
	
	if (vector[2] > thrust_react_z_high) {
		this->thrust += THRUST_STEP;
	} else if (vector[2] < thrust_react_z_high) {
		this->thrust -= THRUST_STEP;
	} else
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
		int * position = this->targetPosition[i].getPosition();
		position[0] += this->formationMovement[0];
		position[1] += this->formationMovement[1];
		position[2] += this->formationMovement[2];
		targetPosition[i].setPosition(position);
	}
}


/*
 * Gehe davon aus dass es ein Topic mit Quadcopters gibt mit URI/Hardware ID 
 * und diese dem quadcopter array zugewiesen wurde qc[id][uri/hardware id]
 */
void Controller::buildFormation()
{
	Position6DOF* formPos = this->formation.getPosition();
	int distance = this->formation.getDistance();
	int * first;
	for(int i = 0; i < amount; i++)
	{
		this->idString = this->quadcopters[i];
		this->id = i;
		this->thrust = THRUST_START;
		this->yawrate = 0;
		this->pitch = 0;
		this->roll = 0;
		int * target = formPos[i].getPosition();
		target[0] *= distance;
		target[1] *= distance;
		target[2] *= distance;
		this->targetPosition[i].setPosition(target);
		move();		
		if( i == 0)
		{
			first = this->currentPosition[0].getPosition();
		}
		else
		{
			//Set all the other positions according to the first crazyflie
			target[0] += first[0];
			target[1] += first[1];
			target[2] += first[2];
			this->targetPosition[i].setPosition(target);
			convertMovement(target);
			//TODO are three coordinate checks too much? Doable? Add epsilon?
			move();
		}
		//Incline a little bit to avoid collisions (there is a level with the qc which are already in position and a moving level)
		target[0] = 0;
		target[1] = distance;
		target[2] = 0;
		convertMovement(target);
		move();		
	}
}

void Controller::shutdownFormation()
{
	for(int i = 0; i < amount; i++)
	{
		this->idString = this->quadcopters[i];
		this->id = i;
		this->thrust = STAND_STILL;
		this->yawrate = 0;
		this->pitch = 0;
		this->roll = 0;
		move();
		//TODO Check for collisions when declining
		this->thrust = THRUST_DECLINE;
		move();
		//TODO Is this point to high?
		this->thrust = THRUST_MIN;
		move();
	}
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
	Position6DOF * formPos;
	for(int i = 0; i < msg->amount; i++)
	{
		int * pos, * ori;
		pos[0] = (int)msg->xPositions[i];
		pos[1] = (int)msg->yPositions[i];
		pos[2] = (int)msg->zPositions[i];
		formPos[i].setPosition(pos);
		//Depends on the calculation of target and current position
		//ori[0] = msg->
		//ori[1] = msg->
		//ori[2] = msg->
		//formPos[i].setOrientation(ori);
	}
	this->formation.setPosition(formPos);
}

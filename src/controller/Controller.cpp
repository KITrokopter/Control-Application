
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

void Controller::initialize()
{
	
}
		
void Controller::makeMovement()
{
	int moveVector[3];
	for(int i = 0; i < amount; i++)
	{		
		control_application::Movement msg;
		msg.id = i;
		moveVector[0] = targetPosition[i].position[0] - currentPosition[i].position[0];
		moveVector[1] = targetPosition[i].position[1] - currentPosition[i].position[1];
		moveVector[2] = targetPosition[i].position[2] - currentPosition[i].position[2];
		msg.thrust = 10;
		msg.yaw = 0.0;
		msg.pitch = 0.0;
		msg.roll = 0.0;
		this->Movement_pub.publish(msg);
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
	this->formation.distance = msg->distance;
	this->formation.amount = msg->amount;
	for(int i = 0; i < amount; i++)
	{
		int * pos, * ori;
		pos[0] = msg->xPositions;
		pos[1] = msg->yPositions;
		pos[2] = msg->zPositions;
		this->formation.position[i].setPosition(msposition.getPosition());
		//ori[0] = msg->
		//ori[1] = msg->
		//ori[2] = msg->
		//this->formation.position[i].setOrientation(position.getOrientation());
	}
}

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
	int moveVector[amount][3];
	for(int i = 0; i < amount; i++)
	{
		moveVector[i][0] = targetPosition[i].position[0] - currentPosition[i].position[0];
		moveVector[i][1] = targetPosition[i].position[1] - currentPosition[i].position[1];
		moveVector[i][2] = targetPosition[i].position[2] - currentPosition[i].position[2];
	}
}

void Controller::MoveFormationCallback(const control_application::MoveFormation::ConstPtr &msg)
{
	ROS_INFO("I heard: %f", msg->xMovement);
}

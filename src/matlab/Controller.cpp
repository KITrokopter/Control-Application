#include "Controller.hpp"

Controller::Controller(OrientedPosition targetPosition[], OrientedPosition currentPosition[], Formation formation)
{
	this.formation = formation;
	this.amount = formation.getDistance();
	for(int i = 0; i < this.amount; i++) 
	{
		this.targetPosition[i] = targetPosition[i];
		this.currentPosition[i] = currentPosition[i];
	}
}

void Controller::initialize()
{
	
}
		
void Controller::makeMovement()
{
	OrientedPosition movement[];
	for(int i = 0; i<this.amout; i++)
	{
		movement[i].x = this.targetPosition[i].x - this.currentPosition[i].x;
		movement[i].y = this.targetPosition[i].y - this.currentPosition[i].y;
		movement[i].z = this.targetPosition[i].z - this.currentPosition[i].z;
		movement[i].xOrientation = this.targetPosition[i].xOrientation - this.currentPosition[i].xOrientation;
		movement[i].yOrientation = this.targetPosition[i].yOrientation - this.currentPosition[i].yOrientation;
		movement[i].zOrientation = this.targetPosition[i].zOrientation - this.currentPosition[i].zOrientation;		
	}
}

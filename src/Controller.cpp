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
		

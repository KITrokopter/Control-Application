#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP
#include "Position6DOF.hpp"
#include "Formation.hpp"
#include "ros/ros.h"
#include "control_application/MoveFormation.h"
#include "control_application/Movement.h"
#include "api_application/SetFormation.h"


class Controller {
public:
	Controller(Position6DOF *targetPosition, Position6DOF *currentPosition, Formation formation);
	void initialize();
	void makeMovement();
	void setTargetPosition(Position6DOF *targetPosition);
	void buildFormation(Formation formation);
	void shutdown();
	void checkInputMovement();
	Position6DOF* getTargetPosition();
protected:
	void MoveFormationCallback(const control_application::MoveFormation::ConstPtr& msg);
	void SetFormationCallback(const api_application::SetFormation::ConstPtr& msg);
private:
	Position6DOF targetPosition[], currentPosition[];
	//Identification der Quadcopters?
	Formation formation;
	int amount;
	//Subscriber for the MoveFormation data of the Quadcopts
	ros::Subscriber MoveFormation_sub;
	//Subscriber for Formation data from API
	ros::Subscriber SetFormation_sub;
	//Publisher for the Movement data of the Quadcopts (1000 is the max. buffered messages)
	ros::Publisher Movement_pub;
	float* formationMovement;
};



#endif // CONTROLLER_HPP

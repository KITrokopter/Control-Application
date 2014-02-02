#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP
#include <math.h>
#include "Position6DOF.hpp"
#include "Formation.hpp"
#include "ros/ros.h"
#include "control_application/MoveFormation.h"	// ? (D)
#include "control_application/Movement.h"		// ? (D)
//TODO How to include when not in the same package
#include "api_application/SetFormation.h"
#include <string>
#include <vector>
#include <pthread.h>

#define THRUST_MIN 10001
#define THRUST_STAND_STILL 18001
#define THRUST_START 22000
#define THRUST_DECLINE 20000
#define THRUST_STEP 50
#define ROLL_STEP 2
#define PITCH_STEP 2
#define INVALID -1

//TODO are three coordinate checks too much? Doable? Add epsilon?
#define POS_CHECK (check[0] != target[0]) || (check[1] != target[1]) || (check[2] != target[2])

class Controller {
public:
	Controller(std::vector<Position6DOF> targetPosition, std::vector<Position6DOF> currentPosition, Formation formation);
	Controller();
	
	void initialize();
	
	void calculateMovement();
	void move();
	void convertMovement(int* vector);
	void setTargetPosition();
	
	//use this as service and then don't use setformation
	//void buildFormation(Formation formation);
	void buildFormation();
	void shutdownFormation();
	void checkInputMovement();
	
	Position6DOF* getTargetPosition();
    
protected:
	void MoveFormationCallback(const control_application::MoveFormation::ConstPtr& msg);
	void SetFormationCallback(const api_application::SetFormation::ConstPtr& msg);

private:
	std::vector<Position6DOF> targetPosition;
	std::vector<Position6DOF> currentPosition;
	//Identification of Quadcopters?
	Formation formation;
	int amount;
	float formationMovement[3];
	std::vector<std::string> quadcopters;
	int thrust;
	float pitch, roll, yawrate;
	std::string idString;
	int id;
	int startProcess;

	//Subscriber for the MoveFormation data of the Quadcopts
	ros::Subscriber MoveFormation_sub;
	//Subscriber for Formation data from API
	ros::Subscriber SetFormation_sub;
	//Publisher for the Movement data of the Quadcopts (1000 is the max. buffered messages)
	ros::Publisher Movement_pub;

};


#endif // CONTROLLER_HPP

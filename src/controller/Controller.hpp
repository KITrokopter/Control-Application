#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP
#include <math.h>
#include "Position6DOF.hpp"
#include "Formation.hpp"
#include "ros/ros.h"
#include "api_application/MoveFormation.h"	// ? (D)
#include "control_application/quadcopter_movement.h"		// ? (D)
#include "api_application/SetFormation.h"
#include "quadcopter_application/quadcopter_status.h"
#include "api_application/BuildFormation.h"
#include "api_application/Shutdown.h"
#include <string>
#include <vector>
#include <pthread.h>
#include "Mutex.hpp"
#include <cmath>

#define THRUST_MIN 10001
#define THRUST_STAND_STILL 18001
#define THRUST_START 22000
#define THRUST_DECLINE 20000
#define THRUST_STEP 50
#define ROLL_STEP 2
#define PITCH_STEP 2
#define INVALID -1

class Controller {
public:
	Controller();

	/* Initializing */
	void initialize();

	/* Movement and Positioning */
	void calculateMovement();
	void sendMovement();
	void convertMovement(double* const vector);
	Position6DOF* getTargetPosition();
	void setTargetPosition();

	/* Formation */
	void buildFormation();
	void shutdownFormation();
	
	void shutdown();
	
	void checkInputMovement();
	
    
protected:
	void MoveFormationCallback(const api_application::MoveFormation::ConstPtr& msg);
	void SetFormationCallback(const api_application::SetFormation::ConstPtr& msg);
	void QuadStatusCallback(const quadcopter_application::quadcopter_status::ConstPtr& msg);

private:
	std::vector<Position6DOF> targetPosition;
	std::vector<Position6DOF> currentPosition;
	//Identification of Quadcopters?
	Formation formation;
	int amount;
	float formationMovement[3];
	std::vector<std::string> quadcopters;
	//Set data
	int thrust;
	float pitch, roll, yawrate;
	//Received data
	std::vector<float> pitch_stab;
	std::vector<float> roll_stab;
	std::vector<float> yaw_stab;
	std::vector<unsigned int> thrust_stab;
	std::vector<float> battery_status;
	std::string idString;
	int id;
	std::vector<bool> tracked;
	bool newTarget;
	bool newCurrent;
	bool shutdownStarted;
	
	Mutex curPosMutex;
	Mutex tarPosMutex;
	Mutex shutdownMutex;

	pthread_t tCalc;
	pthread_t tSend;

	//Subscriber for the MoveFormation data
	ros::Subscriber MoveFormation_sub;
	//Subscriber for Formation data from API
	ros::Subscriber SetFormation_sub;
	//Subscriber for Quadcopter data from QuadcopterModul
	ros::Subscriber QuadStatus_sub;
	//Publisher for the Movement data of the Quadcopts (1000 is the max. buffered messages)
	ros::Publisher Movement_pub;
	//Service for building formation
	ros::ServiceServer BuildForm_srv;
	//Service for shutingdown formation
	ros::ServiceServer Shutdown_srv;

};


#endif // CONTROLLER_HPP

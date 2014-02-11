#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP
#include "Position6DOF.hpp"
#include "Formation.hpp"
#include "ros/ros.h"
#include "api_application/MoveFormation.h"	// ? (D)
#include "control_application/Movement.h"		// ? (D)
#include "../matlab/Vector.h"
#include "control_application/quadcopter_movement.h"		// ? (D)
#include "api_application/SetFormation.h"
#include "quadcopter_application/quadcopter_status.h"
#include "api_application/BuildFormation.h"
#include "api_application/Shutdown.h"
#include <time.h>
#include <<stdio.h>
#include <math.h>
#include <string>
#include <vector>
#include <pthread.h>
#include <list>
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

/* Used for lists */
#define MAX_NUMBER_QUADCOPTER 10

#define POS_CHECK (current[0] != target[0]) || (current[1] != target[1]) || (current[2] != target[2])

class Controller : public IPositionReceiver {
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
	void updatePositions(std::vector<Vector> positions, std::vector<int> ids, std::vector<int> updates);

	/* Formation */
	void buildFormation();
	void shutdownFormation();
	
	void shutdown();
	
	void checkInputMovement();
	
    
protected:
	//Callbacks for Ros subscriber
	void MoveFormationCallback(const api_application::MoveFormation::ConstPtr& msg);
	void SetFormationCallback(const api_application::SetFormation::ConstPtr& msg);
	void QuadStatusCallback(const quadcopter_application::quadcopter_status::ConstPtr& msg);

private:
	/*  */

	/* Position */
	std::vector<Position6DOF> targetPosition;
	std::vector<Position6DOF> currentPosition;
/*	std::vector<rpy-values> sentMovement;*/
	bool listInit;
	std::list<std::vector<Position6DOF> > listPositions;
	std::list<std::vector<Position6DOF> > listTargets;
	std::list<std::vector<Position6DOF> > listSendTargets;
	
	/* Identification of Quadcopters */
	int amount; /* Unknown at start, variable at runtime. */
	Formation formation;
	float formationMovement[3]; /*TODO*/
	
	//Receive data over ROS
	Formation formation;
	//TODO needs to be with service find all
	int totalAmount;
	int amount;
	float formationMovement[3];
	
	//Mapping of int id to string id/ hardware id   qc[id][uri/hardware id]
	std::vector<std::string> quadcopters;
	
	/* Set data */ /*TODO*/
	int thrust;
	float pitch, roll, yawrate;

	/* Received data */ /*TODO*/
	int id;
	std::vector<float> pitch_stab;
	std::vector<float> roll_stab;
	std::vector<float> yaw_stab;
	std::vector<unsigned int> thrust_stab;
	std::vector<float> battery_status;
	std::string idString;
	/*int id;*/
	int startProcess;
/*	int newTarget;*/
/*	int newCurrent;*/
	std::vector<bool> tracked;
/*	bool newTarget;*/
/*	bool newCurrent;*/
	std::vector<string> idString;

	//Control variables
	//Array of tracked quadcopters
	std::vector<bool> tracked;
	//Set when we are in the shutdown process
	bool shutdownStarted;
	
	//Mutex for currentPosition, targetPosition and shutdown
	Mutex curPosMutex;
	Mutex tarPosMutex;
	Mutex shutdownMutex;
	Mutex listPositionsMutex;

	/* Threads */
	pthread_t tCalc;

	/* Subscriber */
	//Subscriber for the MoveFormation data
	ros::Subscriber MoveFormation_sub;
	//Subscriber for Formation data from API
	ros::Subscriber SetFormation_sub;
	//Subscriber for Quadcopter data from QuadcopterModul
	ros::Subscriber QuadStatus_sub;

	//Publisher
	//Publisher for the Movement data of the Quadcopts (1000 is the max. buffered messages)
	ros::Publisher Movement_pub;

	/* Services */
	//Service for building formation
	ros::ServiceServer BuildForm_srv;
	//Service for shutingdown formation
	ros::ServiceServer Shutdown_srv;

};


#endif // CONTROLLER_HPP

#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP
#include <math.h>
#include "Position6DOF.hpp"
#include "Formation.hpp"
#include "ros/ros.h"
#include "api_application/MoveFormation.h"	// ? (D)
#include "control_application/Movement.h"		// ? (D)
#include "api_application/SetFormation.h"
#include "quadcopter_application/quadcopter_status.h"
#include "api_application/BuildFormation.h"
#include "api_application/Shutdown.h"
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

//TODO are three coordinate checks too much? Doable? Add epsilon?
#define POS_CHECK (current[0] != target[0]) || (current[1] != target[1]) || (current[2] != target[2])

class Controller : IPositionReceiver {
public:
	Controller(std::vector<Position6DOF> targetPosition, std::vector<Position6DOF> currentPosition, Formation formation);
	Controller();

	/* Initializing */
	void initialize();

	/* Movement and Positioning */
	void calculateMovement();
	void move();
	void convertMovement(double* const vector);
	Position6DOF* getTargetPosition();
	void setTargetPosition();
	void updatePositions(std::vector<Vector> positions, std::vector<int> ids, std::vector<int> updates) = 0;

	/* Formation */
	//use this as service and then don't use setformation
	//void buildFormation(Formation formation);
	void buildFormation();
	void shutdownFormation();
	
	void shutdown();
	
	void checkInputMovement();
	
    
protected:
	void MoveFormationCallback(const api_application::MoveFormation::ConstPtr& msg);
	void SetFormationCallback(const api_application::SetFormation::ConstPtr& msg);
	void QuadStatusCallback(const quadcopter_application::quadcopter_status::ConstPtr& msg);

private:
	/*  */

	/* Position */
	std::vector<Position6DOF> targetPosition;
	std::vector<Position6DOF> currentPosition;
	std::vector<std::list<Position6DOF>> listPositions;
	std::vector<std::list<Position6DOF>> listTargets;
	std::vector<std::list<Position6DOF>> listSendTargets;
	bool listInit;
	
	//Identification of Quadcopters?

	/*  */
	Formation formation;
	int amount;
	float formationMovement[3];
	std::vector<std::string> quadcopters;
	int thrust;
	float pitch, roll, yawrate;
	std::vector<float> mag[3];
	std::vector<float> gyro[3];
	std::string idString;
	int id;
	int startProcess;
	int newTarget;
	int newCurrent;
	int shutdown;
	
	Mutex curPosMutex;
	Mutex tarPosMutex;
	Mutex shutdownMutex;

	/* Threads */
	std::pthread_t tCalc;
	std::pthread_t tSend;

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

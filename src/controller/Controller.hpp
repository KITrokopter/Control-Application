#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP
#include <math.h>
#include "Position6DOF.hpp"
#include "Formation.hpp"
#include "ros/ros.h"
//Ros messages/services
#include "api_application/MoveFormation.h"	// ? (D)
#include "control_application/quadcopter_movement.h"		// ? (D)
#include "api_application/SetFormation.h"
#include "quadcopter_application/find_all.h"
#include "quadcopter_application/blink.h"
#include "quadcopter_application/quadcopter_status.h"
#include "control_application/BuildFormation.h"
#include "control_application/Shutdown.h"
#include <string>
#include <vector>
#include <pthread.h>
#include "Mutex.hpp"
#include <cmath>
#include <time.h>
#include <sstream>
#include <boost/bind.hpp>

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
	bool buildFormation(control_application::BuildFormation::Request  &req, control_application::BuildFormation::Response &res);
	void shutdownFormation();
	
	bool shutdown(control_application::Shutdown::Request  &req, control_application::Shutdown::Response &res);
	
	void checkInputMovement();
	
    
protected:
	//Callbacks for Ros subscriber
	void MoveFormationCallback(const api_application::MoveFormation::ConstPtr& msg);
	void SetFormationCallback(const api_application::SetFormation::ConstPtr& msg);
	void QuadStatusCallback(const quadcopter_application::quadcopter_status::ConstPtr& msg, int topicNr);

private:
	std::vector<Position6DOF> targetPosition;
	std::vector<Position6DOF> currentPosition;
	
	//Receive data over ROS
	Formation formation;
	//TODO needs to be with service find all
	int totalAmount;
	int amount;
	float formationMovement[3];
	
	//Mapping of int id to string id/ hardware id   qc[id][uri/hardware id]
	std::vector<std::string> quadcopters;
	//Set data
	int thrust;
	float pitch, roll, yawrate;
	int id;
	//Received data
	std::vector<float> pitch_stab;
	std::vector<float> roll_stab;
	std::vector<float> yaw_stab;
	std::vector<unsigned int> thrust_stab;
	std::vector<float> battery_status;
	std::vector<std::string> idString;

	//Control variables
	//Array of tracked quadcopters
	std::vector<bool> tracked;
	//Set when we are in the shutdown process
	bool shutdownStarted;
	
	//Mutex for currentPosition, targetPosition and shutdown
	Mutex curPosMutex;
	Mutex tarPosMutex;
	Mutex shutdownMutex;

	//Threads
	pthread_t tCalc;
	pthread_t tSend;

	/**
  	 * NodeHandle is the main access point to communications with the ROS system.
	 * The first NodeHandle constructed will fully initialize this node, and the last
     * NodeHandle destructed will close down the node.
     */
    ros::NodeHandle n;

	//Subscriber
	//Subscriber for the MoveFormation data
	ros::Subscriber MoveFormation_sub;
	//Subscriber for Formation data from API
	ros::Subscriber SetFormation_sub;
	//Subscriber for Quadcopter data from QuadcopterModul
	//ros::Subscriber QuadStatus_sub;
	std::vector<ros::Subscriber> QuadStatus_sub;

	//Publisher
	//Publisher for the Movement data of the Quadcopts (1000 is the max. buffered messages)
	//ros::Publisher Movement_pub;
	std::vector<ros::Publisher> Movement_pub;

	//Service
	//Service for building formation
	ros::ServiceServer BuildForm_srv;
	//Service for shutingdown formation
	ros::ServiceServer Shutdown_srv;

	//Clients
	ros::ServiceClient FindAll_client;
	ros::ServiceClient Blink_client;

};


#endif // CONTROLLER_HPP

#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP
#include "Position6DOF.hpp"
#include "MovementQuadruple.hpp"
#include "Formation.hpp"
#include "ros/ros.h"
//Ros messages/services
#include "api_application/MoveFormation.h"	// ? (D)
#include "../matlab/Vector.h"
#include "../matlab/TrackingArea.h"
#include "control_application/quadcopter_movement.h"		// ? (D)
#include "api_application/SetFormation.h"
#include "api_application/Message.h"
#include "api_application/Announce.h"
#include "api_application/System.h"
#include "quadcopter_application/find_all.h"
#include "quadcopter_application/blink.h"
#include "quadcopter_application/quadcopter_status.h"
#include "control_application/BuildFormation.h"
#include "control_application/Shutdown.h"
#include "control_application/SetQuadcopters.h"
#include "../position/IPositionReceiver.hpp"
#include <time.h>
#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include <string>
#include <vector>
#include <pthread.h>
#include <list>
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
//TODO 100% = 1?
#define LOW_BATTERY 0.05
//In seconds
#define TIME_UPDATED 1

/* For calculateMovement */
#define CALCULATE_NONE 0 // Unset
#define CALCULATE_START 1
#define CALCULATE_STABILIZE 2
#define CALCULATE_HOLD 3
#define CALCULATE_MOVE 4
#define CALCULATE_ACTIVATED 6 // QC ready to receive data from quadcoptermodul

#define CALCULATE_TAKE_OLD_VALUE -5

/* Used for lists */
#define MAX_NUMBER_QUADCOPTER 10

#define POS_CHECK (current[0] != target[0]) || (current[1] != target[1]) || (current[2] != target[2])

class Controller : public IPositionReceiver {
//class Controller {
public:
	Controller();

	/* Initializing */
	void initialize();

	/* Movement and Positioning */
	void convertMovement(double* const vector);
	Position6DOF* getTargetPosition();
	void setTargetPosition();
	void updatePositions(std::vector<Vector> positions, std::vector<int> ids, std::vector<int> updates);
	void sendMovement();
	void sendMovementAll();
	void calculateMovement();
	void reachTrackedArea(std::vector<int> ids);
	void moveUp(std::vector<int> ids);
	void moveUp( int internId );
	void moveUpNoArg();

	/* Formation also services*/
	bool buildFormation(control_application::BuildFormation::Request  &req, control_application::BuildFormation::Response &res);
	void shutdownFormation();

	/* Service to set Quadcopter IDs*/
	bool setQuadcopters(control_application::SetQuadcopters::Request  &req, control_application::SetQuadcopters::Response &res);
	
	bool shutdown(control_application::Shutdown::Request &req, control_application::Shutdown::Response &res);
	
	bool checkInput();
	void emergencyRoutine(std::string message);
	
    
protected:
	//Callbacks for Ros subscriber
	void MoveFormationCallback(const api_application::MoveFormation::ConstPtr& msg);
	void SetFormationCallback(const api_application::SetFormation::ConstPtr& msg);
	void QuadStatusCallback(const quadcopter_application::quadcopter_status::ConstPtr& msg, int topicNr);
	void SystemCallback(const api_application::System::ConstPtr& msg);
	
	void stopReachTrackedArea();
	void stabilize( int internId );
	void hold( int internId );

private:
	/*  */

	/* Position */
	//std::vector<Position6DOF> targetPosition;
	//std::vector<Position6DOF> currentPosition;
	//std::vector<rpy-values> sentMovement;
	//TODO Where are those lists filled/ updated? FIFO? Latest element last?
	//TODO Include time somehow or how do we check if there hasn't been any input?
	std::list<std::vector<Position6DOF> > listPositions;
	std::list<std::vector<Position6DOF> > listTargets;
	std::list<std::vector<Position6DOF> > listSendTargets;
	
	/* Identification of Quadcopters */
	//Receive data over ROS
	Formation formation;
	//TODO needs to be with service find all
	int totalAmount;
	int amount;
	std::list<float[3]> formationMovement;
	time_t lastFormationMovement;
	time_t lastCurrent;
	unsigned int senderID;
	//TODO Set area
	TrackingArea trackingArea;
	
	//Mapping of int id to string id/ hardware id   qc[id][uri/hardware id]
	//std::vector<std::string> quadcopters;
	//Mapping of quadcopter global id
	std::vector<unsigned int> quadcopters;
	/* For calculateMovement, using local id from mapping before. */
	std::vector<unsigned int> quadcopterMovementStatus;
	
	/* Set data */ 
	int thrust;
	float pitch, roll, yawrate;
	std::vector<MovementQuadruple> movementAll;

	/* Received data */ 
	int id;
	//Arrays for quadcopters sorted by intern id
	std::vector<float> pitch_stab;
	std::vector<float> roll_stab;
	std::vector<float> yaw_stab;
	std::vector<unsigned int> thrust_stab;
	std::vector<float> battery_status;
	int startProcess;
	std::vector<std::string> idString;
	std::vector<int> idsToGetTracked;

	//Control variables
	//Array of tracked quadcopters
	std::vector<bool> tracked;
	//Set when we are in the shutdown process
	bool shutdownStarted;
	bool getTracked;
	bool receivedQuadcopters;
	
	/* Mutex */
	Mutex curPosMutex;
	Mutex tarPosMutex;
	Mutex shutdownMutex;
	Mutex formMovMutex;
	Mutex listPositionsMutex;
	Mutex getTrackedMutex;

	/* Threads */
	pthread_t tCalc;
	pthread_t tSend;
	pthread_t tGetTracked;

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
	//Subscriber to System topic (Sends the start and end of the system)
	ros::Subscriber System_sub;

	/* Publisher */
	//Publisher for the Movement data of the Quadcopts (1000 is the max. buffered messages)
	//ros::Publisher Movement_pub;
	std::vector<ros::Publisher> Movement_pub;
	//Publisher for Message to API
	ros::Publisher Message_pub;

	/* Services */
	//Service for building formation
	ros::ServiceServer BuildForm_srv;
	//Service for shutingdown formation
	ros::ServiceServer Shutdown_srv;
	//Service for setting the quadcopter ids
	ros::ServiceServer QuadID_srv;

	//Clients
	ros::ServiceClient FindAll_client;
	ros::ServiceClient Blink_client;
	ros::ServiceClient Announce_client;

};


#endif // CONTROLLER_HPP

#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP
#include "Formation.hpp"
#include "Interpolator.hpp"
#include "MovementQuadruple.hpp"
#include "Mutex.hpp"
#include "Position6DOF.hpp"
#include "ros/ros.h"
#include "api_application/MoveFormation.h"
#include "api_application/SetFormation.h"
#include "api_application/Message.h"
#include "api_application/Announce.h"
#include "api_application/System.h"
#include "control_application/quadcopter_movement.h"
#include "control_application/BuildFormation.h"
#include "control_application/Shutdown.h"
#include "control_application/SetQuadcopters.h"
//#include "quadcopter_application/find_all.h"
#include "quadcopter_application/blink.h"
#include "quadcopter_application/quadcopter_status.h"
#include "../position/IPositionReceiver.hpp"
#include "../matlab/profiling.hpp"
#include "../matlab/Vector.h"
#include "../matlab/TrackingArea.h"
#include <boost/bind.hpp>
#include <cmath>
#include <list>
#include <math.h>
#include <pthread.h>
#include <sstream>
#include <stdio.h>
#include <string>
#include <unistd.h>
#include <vector>
//Ros messages/services

#define THRUST_MIN 10001
#define THRUST_STAND_STILL 18001
#define THRUST_START 22000
#define THRUST_DECLINE 20000
#define THRUST_STEP 50
#define ROLL_STEP 2
#define PITCH_STEP 2
#define INVALID -1
#define LOW_BATTERY 0.05 //TODO 100% = 1?
#define TIME_UPDATED_END 1*1000*1000 // in ns
#define TIME_UPDATED_CRITICAL 200*1000 // in ns

/* For calculateMovement */
#define CALCULATE_NONE 0 // Unused for formation
#define CALCULATE_START 1 // Send high thrust to reach tracked area
#define CALCULATE_STABILIZE 2 // Tries to hold position (with certain error value)
#define CALCULATE_MOVE 3	// With target and current position
#define CALCULATE_HOLD 4	// Stabilize with more available data, error-handling
#define CALCULATE_LAND 5 //Shutdown quadcopter

#define RANGE_STABLE 0.1 // Distance of two points to be considered "equal" in m
#define RANGE_STABLE_Z 0.08 // Difference of two height-Values to be considered "equal" in m

#define MAX_NUMBER_QUADCOPTER 10 /* Used for lists */

class Formation;
class Controller : public IPositionReceiver {
	
public:
	Controller();

	/* Initializing */
	void initialize();

	/* Movement and Positioning */
	void convertMovement(double* const vector, int internId);
	Position6DOF* getTargetPosition();
	void setTargetPosition();
	void updatePositions(std::vector<Vector> positions, std::vector<int> ids, std::vector<int> updates);
	void sendMovementAll();
	void calculateMovement();
	void land( int internId, int * nrLand  );
	void buildFormation();
	
	/* Formation also services*/
	bool startBuildFormation(control_application::BuildFormation::Request  &req, control_application::BuildFormation::Response &res);
	void shutdownFormation();

	/* Service to set Quadcopter IDs*/
	bool setQuadcopters(control_application::SetQuadcopters::Request  &req, control_application::SetQuadcopters::Response &res);
	
	bool shutdown(control_application::Shutdown::Request &req, control_application::Shutdown::Response &res);
	
	int getLocalId(int globalId);
	bool checkInput();
	void emergencyRoutine(std::string message);
	
    
protected:
	//Callbacks for Ros subscriber
	void MoveFormationCallback(const api_application::MoveFormation::ConstPtr& msg);
	void SetFormationCallback(const api_application::SetFormation::ConstPtr& msg);
	void QuadStatusCallback(const quadcopter_application::quadcopter_status::ConstPtr& msg, int topicNr);
	void SystemCallback(const api_application::System::ConstPtr& msg);
	
	void moveUp();	// move up all
	void moveUp(std::vector<int> ids);	// move up mentioned in ids
	void moveUp( int internId );	// the calculation function
	void stabilize( int internId );
	bool isStable( int internId );
	void hold( int internId );

private:

	/* Position */
	std::vector<std::list<Position6DOF> > listPositions;
	std::vector<std::list<Position6DOF> > listTargets;
	std::vector<std::list<MovementQuadruple> > listSentQuadruples;
	
	/* Identification of Quadcopters */
	//Receive data over ROS
	Formation *formation;
	//int amount;	// Needed for formation
	std::list<std::vector<float> > formationMovement;
	long int lastFormationMovement;
	long int lastCurrent[MAX_NUMBER_QUADCOPTER];
	unsigned int senderID;	
	TrackingArea trackingArea;
	
	//Mapping of quadcopter global id qudcopters[local id] = global id
	std::vector<unsigned long int > quadcopters;
	/* For calculateMovement, using local id from mapping before. */
	std::vector<unsigned int > quadcopterMovementStatus;
	
	/* Set data */ 
	std::vector<MovementQuadruple > movementAll;

	/* Received data */ 
	//Arrays for quadcopters sorted by intern id
	float pitch_stab[MAX_NUMBER_QUADCOPTER];
	float roll_stab[MAX_NUMBER_QUADCOPTER];
	float yaw_stab[MAX_NUMBER_QUADCOPTER];
	unsigned int thrust_stab[MAX_NUMBER_QUADCOPTER];
	float battery_status[MAX_NUMBER_QUADCOPTER];

	/* Control variables */
	bool tracked[MAX_NUMBER_QUADCOPTER]; //Array of tracked quadcopters
	bool landFinished; //Set when we are in the shutdown process
	bool receivedQuadcopters;
	bool receivedFormation;
	bool receivedQuadStatus[MAX_NUMBER_QUADCOPTER];
	bool buildFormationFinished;
	bool buildFormationStop;
	bool shutdownStarted;

	/* 
	 * TODO buildFormationStop/ -Finished, landFinished, ...
	 * TODO thread für shutdown formation
	 * 	Testing @Carina
	 * 
	 * TODO position6dof set timestamp in constructor without timestamp parameter?
	 * /
	
	/* Mutex */
	Mutex shutdownMutex;
	Mutex landMutex;
	Mutex formationMovementMutex;
	Mutex listPositionsMutex;
	Mutex listTargetsMutex;
	Mutex buildFormationMutex;
	Mutex stopFormationMutex;
	Mutex trackedArrayMutex;
	Mutex receivedQCMutex;
	Mutex receivedFormMutex;
	Mutex receivedQCStMutex;
	Mutex lastFormationMovementMutex;
	Mutex lastCurrentMutex;

	/* Threads */
	pthread_t tCalculateMovement;
	pthread_t tBuildFormation;

	/**
  	* NodeHandle is the main access point to communications with the ROS system.
	* The first NodeHandle constructed will fully initialize this node, and the last
	* NodeHandle destructed will close down the node.
	*/
	ros::NodeHandle n;

	/* Subscriber */
	//Subscriber for the MoveFormation data
	ros::Subscriber MoveFormation_sub;
	//Subscriber for Formation data from API
	ros::Subscriber SetFormation_sub;
	//Subscriber for Quadcopter data from QuadcopterModul
	//ros::Subscriber QuadStatus_sub;
	ros::Subscriber QuadStatus_sub[10];
	//Subscriber to System topic (Sends the start and end of the system)
	ros::Subscriber System_sub;

	/* Publisher */
	//Publisher for the Movement data of the Quadcopts (1000 is the max. buffered messages)
	//ros::Publisher Movement_pub;
	ros::Publisher Movement_pub[10];
	//Publisher for Message to API
	ros::Publisher Message_pub;

	/* Services */
	//Service for building formation
	ros::ServiceServer BuildForm_srv;
	//Service for shutingdown formation
	ros::ServiceServer Shutdown_srv;
	//Service for setting the quadcopter ids
	ros::ServiceServer QuadID_srv;

	/* Clients */
	ros::ServiceClient Announce_client;
	ros::ServiceClient Shutdown_client;

};


#endif // CONTROLLER_HPP

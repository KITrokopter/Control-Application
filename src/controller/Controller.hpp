#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP
#include "CalculatorMoveUp.hpp"
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

#define THRUST_MAX_START 48000
#define THRUST_MIN 28000
#define THRUST_SHUTDOWN 0
#define THRUST_STAND_STILL 28001
#define THRUST_START 30000
#define THRUST_DECLINE 200
#define THRUST_MAX 40001
#define THRUST_STEP 200
#define ROLL_MAX 8
#define PITCH_MAX 8
#define YAWRATE_MAX 0
#define INVALID -1
#define LOW_BATTERY 3.0//In V
#define TIME_UPDATED_END 500*1000*1000	// in ns
#define TIME_UPDATED_CRITICAL 200*1000	// in ns
#define TIME_MIN_LOOP_CALC 3000 	// 3 ms for usleep
#define TIME_MIN_CALC 30000000	// 30ms for loop

/* For calculateMovement */
#define CALCULATE_NONE 0 // Unused for formation
#define CALCULATE_START 1 // Send high thrust to reach tracked area
#define CALCULATE_STABILIZE 2 // Tries to hold position (with certain error value)
#define CALCULATE_MOVE 3	// With target and current position
#define CALCULATE_HOLD 4	// Stabilize with more available data, error-handling
#define CALCULATE_LAND 5 //Shutdown quadcopter

#define CALCULATE_STABILIZE_STEP 500	// time in ms after next value should be calculated

#define HOLD_SKIP 1
#define HOLD_LENGTH 1000	// time in ms to stay in function before "shutdown"
#define HOLD_FACTOR_THRUST 0.7
#define HOLD_FACTOR_RPY 0.5

#define RANGE_STABLE 10 // Distance of two points to be considered "equal" in mm
#define RANGE_STABLE_Z 6 // Difference of two height-Values to be considered "equal" in mm

#define MAX_NUMBER_QUADCOPTER 10 /* Used for lists */
#define MAX_SAVED_SENT_QUADRUPLES 8

class Formation;
class Controller : public IPositionReceiver {
	
public:
	Controller();

	/* Initializing */
	void initialize();
	void setTrackingArea(TrackingArea area);

	/* Movement and Positioning */
	//Position6DOF* getTargetPosition();
	void setTargetPosition();
	void updatePositions(std::vector<Vector> positions, std::vector<int> ids, std::vector<int> updates);
	void sendMovementAll();
	void calculateMovement();
	void buildFormation();
	
	/* Formation */
	bool startBuildFormation(control_application::BuildFormation::Request  &req, control_application::BuildFormation::Response &res);

	/* Service to set Quadcopter IDs*/
	bool setQuadcopters(control_application::SetQuadcopters::Request  &req, control_application::SetQuadcopters::Response &res);

	/* Shutdown */
	bool shutdown(control_application::Shutdown::Request &req, control_application::Shutdown::Response &res);
	void shutdownFormation();

	/* Helper functions */
	int getLocalId(int globalId);
	bool checkInput(int internId);
	void emergencyRoutine(std::string message);	
    
protected:
	//Callbacks for Ros subscriber
	void MoveFormationCallback(const api_application::MoveFormation::ConstPtr& msg);
	void SetFormationCallback(const api_application::SetFormation::ConstPtr& msg);
	void QuadStatusCallback(const quadcopter_application::quadcopter_status::ConstPtr& msg, int topicNr);
	void SystemCallback(const api_application::System::ConstPtr& msg);
	
	void dontMove( int internId);
	void moveUp( int internId );
	void stabilize( int internId );
	void land( int internId, int * nrLand  );
	void hold( int internId );

	/* Helper functions */
	bool isStable( int internId );
private:
	
	/* static data */	
	Formation *formation;	//Receive data over ROS
	unsigned int senderID;	//Receive data over ROS	
	TrackingArea trackingArea;
	Interpolator interpolator;
	
	//Mapping of quadcopter global id qudcopters[local id] = global id
	std::vector<unsigned long int > quadcopters;
	/* For calculateMovement, using local id from mapping before. */
	std::vector<unsigned int > quadcopterMovementStatus;
	long int time;
	long int time2;
	long int time3;
	unsigned int thrustTest;	
	/* Position */
	std::vector<std::list<Position6DOF> > listPositions;
	std::vector<std::list<Position6DOF> > listTargets;
	
	/* Set data */ 
	std::vector<std::list<MovementQuadruple> > listSentQuadruples;
	std::vector<std::list<MovementQuadruple> > listFutureMovement;

	/* Received data */ 
	//Arrays for quadcopters sorted by intern id
	float pitch_stab[MAX_NUMBER_QUADCOPTER];
	float roll_stab[MAX_NUMBER_QUADCOPTER];
	float yaw_stab[MAX_NUMBER_QUADCOPTER];
	unsigned int thrust_stab[MAX_NUMBER_QUADCOPTER];
	float battery_status[MAX_NUMBER_QUADCOPTER];
	std::list<std::vector<float> > formationMovement;

	/* Control variables */
	bool tracked[MAX_NUMBER_QUADCOPTER]; //Array of tracked quadcopters
	bool landFinished; //Set when we are in the shutdown process
	bool receivedQuadcopters;
	bool receivedFormation;
	bool receivedQuadStatus[MAX_NUMBER_QUADCOPTER];
	bool buildFormationFinished;
	bool receivedTrackingArea;
	bool shutdownStarted;
	long int lastFormationMovement;
	long int lastCurrent[MAX_NUMBER_QUADCOPTER];
	
	/* Mutex */
	//Mutex shutdownMutex;
	//Mutex landMutex;
	Mutex formationMovementMutex;
	Mutex listPositionsMutex;
	Mutex listTargetsMutex;
	//Mutex buildFormationMutex;
	//Mutex trackedArrayMutex;
	//Mutex receivedQCMutex;
	//Mutex receivedFormMutex;
	//Mutex receivedQCStMutex;
	Mutex lastFormationMovementMutex;
	Mutex lastCurrentMutex;
	//Mutex movementStatusMutex;

	/* Threads */
	pthread_t tCalculateMovement;
	pthread_t tBuildFormation;
	pthread_t tShutdownFormation;

	/**
  	* NodeHandle is the main access point to communications with the ROS system.
	* The first NodeHandle constructed will fully initialize this node, and the last
	* NodeHandle destructed will close down the node.
	*/
	ros::NodeHandle n;

	/* Subscriber */	
	ros::Subscriber MoveFormation_sub;	//Subscriber for the MoveFormation data	
	ros::Subscriber SetFormation_sub;	//Subscriber for Formation data from API
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

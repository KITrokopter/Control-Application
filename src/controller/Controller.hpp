#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP
//#include "CalculatorMoveUp.hpp"
#include "Control.hpp"
#include "PControl.hpp"
#include "Formation.hpp"
#include "Interpolator.hpp"
#include "MovementQuadruple.hpp"
#include "MovementHelper.hpp"
#include "Mutex.hpp"
#include "Position6DOF.hpp"
#include "QuadcopterControl.hpp"
#include "QuadcopterInfo.hpp"
#include "QuadcopterThrust.hpp"
#include "ros/ros.h"
#include "api_application/MoveFormation.h"
#include "api_application/SetFormation.h"
#include "api_application/Message.h"
#include "api_application/Announce.h"
#include "api_application/System.h"
#include "control_application/quadcopter_movement.h"	// @Carina ???
#include "control_application/BuildFormation.h"
#include "control_application/Shutdown.h"
#include "control_application/SetQuadcopters.h"
#include "control_application/Rotation.h"
#include "quadcopter_application/quadcopter_status.h"
#include "../position/IPositionReceiver.hpp"
#include "../matlab/profiling.hpp"
#include "../matlab/Vector.h"
#include "../matlab/TrackingArea.h"
#include "../matlab/Matrix2x2.h"
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

#define INVALID -1

#define USE_BATTERY_INPUT 1

#define AMPLIFICATION_FACTOR_RP 0.001
#define AMPLIFICATION_FACTOR_THRUST 0.01

#define TIME_UPDATED_END 500*1000*1000	// in ns
#define TIME_UPDATED_CRITICAL 200*1000	// in ns
#define TIME_WAIT_FOR_DATA 8000
#define TIME_WAIT_FOR_TRACKED 3000
#define TIME_WAIT_FOR_LANDING 10000
#define TIME_LOOPS_PER_SECOND 30
#define TIME_ROTATE_CIRCLE 12000000000	// 12s for one whole rotation

#define DISTANCE_ROTATE_TO_CENTER 100

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
	void sendMovement( int internId);
	void calculateMovement();
	void buildFormation();
	
	/*Service to rotate formation*/
	bool rotateFormation(control_application::Rotation::Request  &req, control_application::Rotation::Response &res);
	void rotate();
	
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
	int searchNeighbor( double * target, bool * ids);
    
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
//	bool isStable( int internId );

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
	long int timeOffsetOutput;
	long int timeDurationMoveup;
	long int timeOffsetChangeThrust;

	/* Position */
	std::vector<std::list<Position6DOF> > listPositions; //newest Position at the end of the list
	std::vector<std::list<Position6DOF> > listTargets; //newest Position at the end of the list
	
	/* Set data */ 
	std::vector<std::list<MovementQuadruple> > listSentQuadruples; //last send Movement at the end of the list
	std::vector<MovementQuadruple> listCurrentMovement;

	/* Received data */ 
	//Arrays for quadcopters sorted by intern id
	float pitch_stab[MAX_NUMBER_QUADCOPTER];
	float roll_stab[MAX_NUMBER_QUADCOPTER];
	float yaw_stab[MAX_NUMBER_QUADCOPTER];
	unsigned int thrust_stab[MAX_NUMBER_QUADCOPTER];
	float battery_status[MAX_NUMBER_QUADCOPTER];
	std::list<std::vector<float> > formationMovement;

	/* Control */
	Control *controlThrust, *controlRollPitch;
	QuadcopterControl quadcopterStatus[MAX_NUMBER_QUADCOPTER];


	/* Control variables */
	bool tracked[MAX_NUMBER_QUADCOPTER]; //Array of tracked quadcopters
	bool landFinished; //Set when we are in the shutdown process
	bool receivedQuadcopters;
	bool receivedFormation;
	bool receivedQuadStatus[MAX_NUMBER_QUADCOPTER];
	bool buildFormationFinished;
	bool receivedTrackingArea;
	bool shutdownStarted;
	bool rotationInProcess;
	long int timeRotationStarted;
	long int timeLastFormationMovement;
	long int timeLastCurrent[MAX_NUMBER_QUADCOPTER];
	unsigned int thrustHelp[MAX_NUMBER_QUADCOPTER];
	
	/* Mutex */
	Mutex formationMovementMutex;
	Mutex listPositionsMutex;
	Mutex listTargetsMutex;
	Mutex lastFormationMovementMutex;
	Mutex lastCurrentMutex;

	/* Threads */
	pthread_t tCalculateMovement;
	pthread_t tBuildFormation;
	pthread_t tShutdownFormation;
	pthread_t tRotation;

	/**
  	* NodeHandle is the main access point to communications with the ROS system.
	* The first NodeHandle constructed will fully initialize this node, and the last
	* NodeHandle destructed will close down the node.
	*/
	ros::NodeHandle n;

	/* Subscriber */	
	ros::Subscriber MoveFormation_sub;	//Subscriber for the MoveFormation data	
	ros::Subscriber SetFormation_sub;	//Subscriber for Formation data from API
	ros::Subscriber QuadStatus_sub[10];
	ros::Subscriber System_sub;	//Subscriber to System topic (Sends the start and end of the system)
	//ros::Subscriber QuadStatus_sub;	//Subscriber for Quadcopter data from QuadcopterModul

	/* Publisher */
	//Publisher for the Movement data of the Quadcopts (1000 is the max. buffered messages)
	ros::Publisher Movement_pub[10];	//ros::Publisher Movement_pub;
	ros::Publisher Message_pub;	//Publisher for Message to API

	/* Services */

	ros::ServiceServer BuildForm_srv;	//Service for building formation
	ros::ServiceServer Shutdown_srv;	//Service for shutingdown formation
	ros::ServiceServer QuadID_srv;	//Service for setting the quadcopter ids
	ros::ServiceServer Rotation_srv;	//Service for rotating

	/* Clients */
	ros::ServiceClient Announce_client;
	ros::ServiceClient Shutdown_client;

};


#endif // CONTROLLER_HPP

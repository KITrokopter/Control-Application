#ifndef INTERPOLATOR_HPP
#define INTERPOLATOR_HPP

#include <cmath>
#include <list>
#include <time.h>
#include <sys/time.h>
#include "InterpolatorInfo.hpp"
#include "MovementQuadruple.hpp"
#include "Position6DOF.hpp"
#include "QuadcopterThrust.hpp"
#include "ros/ros.h"
#include "../matlab/profiling.hpp"
//#define MAX_NUMBER_QUADCOPTER_HIGH 10
#define MIN_TIME_TO_WAIT 500000000 	// in ns, only for rpy-values
#define PREDICT_FUTURE_POSITION_TIME 200000000	// in ns
#define PREDICT_FUTURE_POSITION_STEPS 1 	// Interpolate with ? number of latest positions (unused)

#define ROTATIONAL_CORRECTION 0	// 0-no correction; 1-always take new value; linear in between wouldn't work
#define POSITIONED_NORTH_AT_START 0	// 0 or 1, used as bool, North means pitch+ -> y+

#define REACHING_TARGET_DIFF 0.6 // Factor 0 <= x <= 1
#define REACHING_HEIGHT_DIFF 0.6 // Factor 0 <= x <= 1

#define SPEED_MIN_INCLINING 0 	// in mm/s
#define SPEED_MAX_INCLINING 8000 	// in mm/s
#define SPEED_MIN_DECLINING -0 	// in mm/s
#define SPEED_MAX_DECLINING -8000 	// in mm/s
#define SPEED_MIN_PLANE 80		// in mm/s
#define SPEED_MAX_PLANE 300		// in mm/s

#define DISTANCE_CLOSE 10 	// in mm
#define DISTANCE_HIGH 100

#define TEST_ROLL_PITCH 0

/*
 * thursday
 * TODO hold
 * TODO Coordinatesystem test
 * TODO roll/pitch-test QC
 * TODO flight of circles, rotation, ...
 * 
 * TODO list future movement - fix
 * TODO use of timestamps
 * TODO ROS_INFO, ROS_WARN, ROS_ERROR, ROS_DEBUG - correct use
 */

class Interpolator {
public:
	Interpolator();
	//MovementQuadruple calibrate(int id, std::list<MovementQuadruple> sentQuadruples);
	MovementQuadruple calculateNextMQ(std::list<MovementQuadruple> &sentQuadruples, std::list<Position6DOF> &positions, Position6DOF &target, QuadcopterThrust thrustInfo, int id);
	MovementQuadruple calculateHold(std::list<MovementQuadruple> &sentQuadruples, std::list<Position6DOF> &positions, QuadcopterThrust thrustInfo, int id);

protected:
	void checkState( int id );

private:
	InterpolatorInfo status[MAX_NUMBER_QUADCOPTER];
	double stepSizeOfChange; 	// depends on the distance of position to target	
	long int timeDiff1;
	long int timeDiff2;
	long int timeDiff3;
	long int aTimeSwitch;
};

#endif // INTERPOLATOR_HPP

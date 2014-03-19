#ifndef INTERPOLATOR_HPP
#define INTERPOLATOR_HPP

#include <cmath>
#include <list>
#include <time.h>
#include <sys/time.h>
#include "MovementQuadruple.hpp"
#include "Position6DOF.hpp"
#include "../matlab/profiling.hpp"
#include "ros/ros.h"

#define MAX_STEPS_IN_ADVANCE 1 	// How delayed is the input?
#define MAX_NUMBER_OF_QUADCOPTER_HIGH 10
#define MIN_TIME_TO_WAIT 500*1000*1000 // in ns

#define REACHING_TARGET_DIFF 0.6 // Factor 0 <= x <= 1
#define REACHING_HEIGHT_DIFF 0.6 // Factor 0 <= x <= 1

#define SPEED_MIN_INCLINING 150 	// in mm/s
#define SPEED_MAX_INCLINING 500 	// in mm/s
#define SPEED_MIN_DECLINING -150 	// in mm/s
#define SPEED_MAX_DECLINING -500 	// in mm/s
#define SPEED_MIN_PLANE 80	// in mm/s
#define SPEED_MAX_PLANE 300	// in mm/s

#define DISTANCE_CLOSE_TO_TARGET 50 	// in mm



/*
 * TODO start depeding on charge
 * 
 * TODO linear interpolation
 * 	TODO same in 3D-view
 * 
 * TODO hold
 * 
 * TODO test of speed, either optimize or save data-to-sent in advance
 * TODO optimize interpolation (step size time-dependant)
 * TODO optimize interpolation (replace linear function)
 * 
 */

class Interpolator {
public:
	Interpolator();

	MovementQuadruple calibrate(int id, std::list<MovementQuadruple> sentQuadruples);
	MovementQuadruple calculateNextMQ(std::list<MovementQuadruple> sentQuadruples, std::list<Position6DOF> positions, Position6DOF target, int id);

protected:

private:

	long int lastUpdated[MAX_NUMBER_OF_QUADCOPTER_HIGH];
	double northeast[MAX_NUMBER_OF_QUADCOPTER_HIGH][2];		// 0-pitch, 1-roll
	double stepSizeOfChange; 	// depends on the distance of position to target

	long int started[MAX_NUMBER_OF_QUADCOPTER_HIGH];

};

#endif // INTERPOLATOR_HPP

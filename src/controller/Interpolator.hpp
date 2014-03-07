#ifndef INTERPOLATOR_HPP
#define INTERPOLATOR_HPP

#include <cmath>
#include <time.h>
#include <sys/time.h>
#include <MovementQuadruple.hpp>

#define MAX_STEPS_IN_ADVANCE 1 	// How delayed is the input?
#define MAX_NUMBER_OF_QUADCOPTER_HIGH 10
#define MIN_TIME_TO_WAIT 500 // in ms

/*
 * TODO linear interpolation
 * TODO test of speed, either optimize or save data-to-sent in advance
 * TODO optimize interpolation (step size time-dependant)
 * TODO optimize interpolation (replace linear function)
 * 
 */

class Interpolator {
public:
	Interpolator();

	MovementQuadruple calculateNextMQ(std::list<MovementQuadruple> sentQuadruples, std::list<Position6DOF> positions, int id);
/*	MovementQuadruple calculateNextMQ(std::list<MovementQuadruple> sentQuadruples, std::list<Position6DOF> positions);*/

protected:

private:

	time_t lastUpdated[MAX_NUMBER_OF_QUADCOPTER_HIGH];
	double speedOfChange; 	// depends on the distance of position to target
	

};

#endif // INTERPOLATOR_HPP

#ifndef INTERPOLATOR_HPP
#define INTERPOLATOR_HPP

#include <cmath>
#include <time.h>
#include <MovementQuadruple.hpp>

#define MAX_STEPS_IN_ADVANCE 1 	// How delayed is the input?

/*
 * TODO safe sent movementQuadruples in lists/ vectors ~ 10times/sec
 * TODO give that data + positional information (latest x datasets?) to calculateNextMQ()
 * TODO linear interpolation
 * TODO test of speed, either optimize or save data-to-sent in advance
 * TODO optimize interpolation (step size time-dependant)
 * TODO optimize interpolation (replace linear function)
 * 
 */

class Interpolator {
public:
	Interpolator();

	MovementQuadruple calculateNextMQ(std::list<MovementQuadruple> sentQuadruples, std::list<Position6DOF> positions);
/*	MovementQuadruple calculateNextMQ(std::list<MovementQuadruple> sentQuadruples, std::list<Position6DOF> positions);*/

protected:

private:

	double speedOfChange; 	// depends on the distance of position to target
	

};

#endif // INTERPOLATOR_HPP

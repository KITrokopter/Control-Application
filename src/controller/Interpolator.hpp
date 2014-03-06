#ifndef INTERPOLATOR_HPP
#define INTERPOLATOR_HPP

#include <time.h>
#include <cmath>

#define MAX_STEPS_IN_ADVANCE 1 	// How delayed is the input?

class Interpolator {
public:
	Interpolator(){};

protected:

private:

	double speedOfChange; 	// depends on the distance of position to target
	

};

#endif // INTERPOLATOR_HPP

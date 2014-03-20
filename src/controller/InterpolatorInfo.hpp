#ifndef INTERPOLATORINFO_HPP
#define INTERPOLATORINFO_HPP

#include <cmath>
#include <list>
#include <time.h>
#include <sys/time.h>

/*
 * TODO 3D-view
 * 
 * TODO hold
 * 
 * TODO test of speed, either optimize or save data-to-sent in advance
 * TODO optimize interpolation (step size time-dependant)
 * TODO optimize interpolation (replace linear function)
 * 
 */

class InterpolatorInfo {
public:
	
	InterpolatorInfo();

protected:

private:

	long int started;
	int state;	
	
	long int lastUpdated;	// in latest state
	double northeast[2];		// 0-x-roll, 1-y-pitch

};

#endif // INTERPOLATORINFO_HPP

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

	/* Getter and Setter */
	long int getStarted();
	void setStarted( long int newStarted );
	int getState();
	void setState( int newState );
	long int getLastUpdated();
	void setLastUpdated( long int newLastUpdated );
	double getX();
	double getY();
	void setX( double newX );
	void setY( double newY );

protected:

private:

	/*
	 * States
	 * 0 not started
	 * 1 started, sent values for roll/pitch
	 * 2 roll/pitch has been sent, calculate x/y values
	 * 3 calibration done, now do only small adjustments
	 */
	int state;	
	long int started;
	
	long int lastUpdated;	// in latest state
	double x;		// 0-x-roll, 1-y-pitch
	double y;

};

#endif // INTERPOLATORINFO_HPP

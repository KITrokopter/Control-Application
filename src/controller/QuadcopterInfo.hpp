/*
 * QuadcopterInfo.hpp
 *
 *  Created on: 30.03.2014
 *      Author: dwx
 */
#ifndef QUADCOPTERINFO_HPP_
#define QUADCOPTERINFO_HPP_

#include <cmath>
#include <list>
#include <time.h>
#include <sys/time.h>

#define UNSTARTED 0
#define STARTED 1
#define CALC 2
#define DONE 3
#define HOLD 4
#define SHUTDOWN 5

class QuadcopterInfo {
public:
	QuadcopterInfo();

	/* Getter and Setter */
	short getState();
	void setState( short newState );

	long int getStarted();
	void setStarted( long int newStarted );
	long int getLastUpdated();
	void setLastUpdated( long int newLastUpdated );
	long int getShutdownStarted();
	void setShutdownStarted( long int newShutdownStarted );

	double getX();
	double getY();
	double getRotation();
	void setX( double newX );
	void setY( double newY );
	void setRotation( double newRotation );
	bool isNegativeSign();
	void setNegativeSign(bool negativeSign);

protected:

private:

	/*
	 * States
	 * 0 not started
	 * 1 started
	 * 2
	 * 3
	 * 4 hold (before shutdown), "smart" shutdown
	 * 5 shutdown (turn off)
	 */
	short state;

	long int timeStarted;
	long int timeLastUpdated;	// used in latest state "DONE", before only "started" is relevant
	long int timeShutdownStarted;

	double x;		// 0-x-roll, 1-y-pitch
	double y;
	double rotation;	// in rad; compared to "North" (pitch=1), counterclockwise
	bool negativeSign;
};

#endif /* QUADCOPTERINFO_HPP_ */

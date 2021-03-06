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

#define ROLL_MIN -12.0
#define ROLL_MAX 12.0
#define ROLL_START 12.0
#define PITCH_MIN -12.0
#define PITCH_MAX 12.0
#define PITCH_START 0.0
#define YAWRATE_MIN -12.0
#define YAWRATE_MAX 12.0
#define YAWRATE_START 0.0

/**
 * A class using central global values for roll, pitch and yawrate.
 * Its functions are currently unused besides checkAndFix_() due to the development of this project.
 * The set global values are the given boundary to prevent from unuseful correctional movement (e.g. a loop).
 */
class QuadcopterInfo {
public:
	QuadcopterInfo();

	/* Getter and Setter */
	short getState();
	void setState(short newState);

	long int getStarted();
	void setStarted(long int newStarted);
	long int getLastUpdated();
	void setLastUpdated(long int newLastUpdated);
	long int getShutdownStarted();
	void setShutdownStarted(long int newShutdownStarted);

	float checkAndFixRoll(float roll);
	float checkAndFixPitch(float pitch);
	float checkAndFixYawrate(float yawrate);

	double getX();
	double getY();
	double getRotation();
	void setX(double newX);
	void setY(double newY);
	void setRotation(double newRotation);
	bool isNegativeSign();
	void setNegativeSign(bool negativeSign);

protected:

private:

	/**
	 * States
	 * 0 not started
	 * 1 started
	 * 2 ?
	 * 3 ?
	 * 4 hold (before shutdown), "smart" shutdown
	 * 5 shutdown (turn off)
	 */
	short state;

	long int timeStarted;
	long int timeLastUpdated;   // used in latest state "DONE", before only
	                            // "started" is relevant
	long int timeShutdownStarted;

	double x;       // 0-x-roll, 1-y-pitch
	double y;
	double rotation;    // in rad; compared to "North" (pitch=1),
	                    // counterclockwise
	bool negativeSign;
};

#endif /* QUADCOPTERINFO_HPP_ */

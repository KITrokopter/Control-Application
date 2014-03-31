#ifndef POSITION6DOF_HPP
#define POSITION6DOF_HPP

#include <time.h>
#include <cmath>
#include "ros/ros.h"
#include "../matlab/profiling.hpp"
#include "../matlab/Vector.h"

class Position6DOF {
public:
	//both arrays of the return pointer have size three
	double* getPosition();
	double* getOrientation();
	void setPosition(double * position);
	void setOrientation(double* orientation);
	long int getTimestamp();
	void setTimestamp(long int newTimestamp);
	Position6DOF(double x, double y, double z, double xOrientation, double yOrientation, double zOrientation);
	Position6DOF(double x, double y, double z);
	Position6DOF(Vector vector);
	Position6DOF(){};

	double getAbsoluteDistance();
	double getAbsoluteDistance( Position6DOF otherPosition );
	double getAbsoluteDistanceXY( Position6DOF otherPosition );
	double getDistanceX( Position6DOF otherPosition ); // positive if "this" is to the left of "otherPosition"
	double getDistanceY( Position6DOF otherPosition ); // positive if "this" is before "otherPosition"
	double getDistanceZ( Position6DOF otherPosition ); // positive if "this" is below "otherPosition"
	void predictNextPosition( Position6DOF olderPosition, long int timeInFuture );

private:
	double position[3];
	double orientation[3];
	long int timestamp;
};



#endif // POSITION6DOF_HPP

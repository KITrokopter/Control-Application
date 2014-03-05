#ifndef POSITION6DOF_HPP
#define POSITION6DOF_HPP
#include <time.h>
#include <cmath>


class Position6DOF {
public:
	//both arrays of the return pointer have size three
	double* getPosition();
	double* getOrientation();
	void setPosition(double * position);
	void setOrientation(double* orientation);
	time_t getTimestamp();
	void setTimestamp(time_t newTimestamp);
	Position6DOF(double x, double y, double z, double xOrientation, double yOrientation, double zOrientation);
	Position6DOF(double x, double y, double z);
	Position6DOF(){};

	double getAbsoluteDistance( Position6DOF otherPosition );

private:
	double position[3];
	double orientation[3];
	time_t timestamp;
};



#endif // POSITION6DOF_HPP

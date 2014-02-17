#ifndef POSITION6DOF_HPP
#define POSITION6DOF_HPP
#include <time.h>


class Position6DOF {
public:
	//both arrays of the return pointer have size three
	double* const getPosition();
	double* const getOrientation();
	void setPosition(double * const position);
	void setOrientation(double* const orientation);
	time_t getTimestamp();
	void setTimestamp(time_t newTimestamp);
	Position6DOF(double x, double y, double z, double xOrientation, double yOrientation, double zOrientation);
	Position6DOF(double x, double y, double z);
	Position6DOF(){};

private:
	double position[3];
	double orientation[3];
	time_t timestamp;
};



#endif // POSITION6DOF_HPP

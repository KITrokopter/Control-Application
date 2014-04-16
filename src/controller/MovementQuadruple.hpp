#ifndef MOVEMENT_QUADRUPLE_HPP
#define MOVEMENT_QUADRUPLE_HPP
#include "../matlab/profiling.hpp"
#include <sys/time.h>
#include "ros/ros.h"
#include "../matlab/profiling.hpp"

/**
 * A movement quadruple contains four values: thrust, roll, pitch and yawrate.
 * It is used to save sent data, data to send and received data.
 */
class MovementQuadruple {
public:
	MovementQuadruple(unsigned int newThrust, float newRoll, float newPitch, float newYawrate);
	MovementQuadruple(unsigned int newThrust, float newRoll, float newPitch, float newYawrate, long int newTimestamp);

	unsigned int getThrust();
	void setThrust(unsigned int newThrust);
	float getRoll();
	float getPitch();
	float getYawrate();
	void setRollPitchYawrate(float newRoll, float newPitch, float newYawrate);
	void setRollPitchYawrate(MovementQuadruple toCopy);

	long int getTimestamp();
	void setTimestamp(long int newTimestamp);

	bool checkQuadruple(int maxThrust, float maxRoll, float maxPitch, float maxYawrate);

private:
	unsigned int thrust;
	float roll, pitch, yawrate;
	long int timestamp;
};

#endif // MOVEMENT_QUADRUPLE_HPP

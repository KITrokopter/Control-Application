#ifndef MOVEMENT_QUADRUPLE_HPP
#define MOVEMENT_QUADRUPLE_HPP

#include <sys/time.h>

class MovementQuadruple
{	
	public:
		MovementQuadruple(unsigned int newThrust, float newRoll, float newPitch, float newYawrate);
		MovementQuadruple(unsigned int newThrust, float newRoll, float newPitch, float newYawrate, long int newTimestamp);
		void setThrust( unsigned int newThrust );
		void setRollPitchYawrate(float newRoll, float newPitch, float newYawrate);
		unsigned int getThrust();
		float getRoll();
		float getPitch();
		float getYawrate();
		
		long int getTimestamp();
		void setTimestamp( long int newTimestamp );

	private:
		unsigned int thrust;
		float roll, pitch, yawrate;
		long int timestamp;

};

#endif // MOVEMENT_QUADRUPLE_HPP

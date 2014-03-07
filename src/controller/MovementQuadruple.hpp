#ifndef MOVEMENT_QUADRUPLE_HPP
#define MOVEMENT_QUADRUPLE_HPP

#include <sys/time.h>

class MovementQuadruple
{	
	public:
		MovementQuadruple(int newThrust, float newRoll, float newPitch, float newYawrate);
		MovementQuadruple(int newThrust, float newRoll, float newPitch, float newYawrate, time_t newTimestamp)
		void setThrust( int newThrust );
		void setRollPitchYawrate(float newRoll, float newPitch, float newYawrate);
		int getThrust();
		float getRoll();
		float getPitch();
		float getYawrate();
		
		long int getTimestamp();
		void setTimestamp( long int newTimestamp );

	private:
		int thrust;
		float roll, pitch, yawrate;
		time_t timestamp;

};

#endif // MOVEMENT_QUADRUPLE_HPP

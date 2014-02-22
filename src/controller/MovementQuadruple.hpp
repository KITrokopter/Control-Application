#ifndef MOVEMENT_QUADRUPLE_HPP
#define MOVEMENT_QUADRUPLE_HPP

class MovementQuadruple
{	
	public:
		MovementQuadruple(int newThrust, float newRoll, float newPitch, float newYawrate);
		void setThrust( int newThrust );
		void setRollPitchYawrate(float newRoll, float newPitch, float newYawrate);
		int getThrust();
		float getRoll();
		float getPitch();
		float getYawrate();

	private:
		int thrust;
		float roll, pitch, yawrate;

};

#endif // MOVEMENT_QUADRUPLE_HPP
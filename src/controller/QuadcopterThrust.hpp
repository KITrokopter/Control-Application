#ifndef QUADCOPTER_THRUST_HPP
#define QUADCOPTER_THRUST_HPP

#include <sys/time.h>
#include <map>

#define QUADCOPTER_THRUST_RANGE 10000 
#define BATTERY_MAX 8
#define BATTERY_LOW 1

class QuadcopterThrust
{	
	public:
		QuadcopterThrust();
		bool initDone();
		bool checkAndSetBatteryValue( float battery );	
		unsigned int checkAndFix( unsigned int thrust );

		void setMin( unsigned int min );
		unsigned int getMin();
		void setMax( unsigned int max );
		unsigned int getMax();
		void setStartMax( unsigned int startMax );
		unsigned int getStartMax();
		void setStart( unsigned int start );
		unsigned int getStart();

	protected:		
		void setThrust( float battery );

	private:
		/* 
		 * All values should be seen Quadcopter-specific.
		 * Global min/max values will be in Controller.hpp as global variables.
		 */
		unsigned int min;
		unsigned int max;
		unsigned int startMax;
		unsigned int start;

		bool setThrustCalled;
};

#endif // QUADCOPTER_THRUST_HPP

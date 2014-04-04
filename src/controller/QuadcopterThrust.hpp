#ifndef QUADCOPTER_THRUST_HPP
#define QUADCOPTER_THRUST_HPP

#include <sys/time.h>
#include <map>
#include "ros/ros.h"

#define BATTERY_MAX 8
#define BATTERY_MIN 1

#define BATTERY_LOW 3.0//In V

#define THRUST_GLOBAL_MAX 60000
#define THRUST_GLOBAL_MIN 10001
#define THRUST_OFF 0
#define THRUST_OFFSET_LOW 26000

#define QUADCOPTER_THRUST_RANGE 10000


class QuadcopterThrust
{	
	public:
		QuadcopterThrust();
		void init();
		bool initDone();
		bool checkAndSetBatteryValue( float battery );	
		void setWithoutBatteryValue();
		unsigned int checkAndFix( unsigned int thrust );
		unsigned int checkAndFix( double thrust);

		void setMin( unsigned int min );
		unsigned int getMin();
		void setMax( unsigned int max );
		unsigned int getMax();
		void setStartMax( unsigned int startMax );
		unsigned int getStartMax();
		void setStart( unsigned int start );
		unsigned int getStart();
		void setDecline( unsigned int decline );
		unsigned int getDecline();

		void setOffset( float battery );
		unsigned int getOffset();

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
		unsigned int decline;

		unsigned int offset;

		bool setThrustCalled;
};

#endif // QUADCOPTER_THRUST_HPP

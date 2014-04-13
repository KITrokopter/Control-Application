#ifndef QUADCOPTER_THRUST_HPP
#define QUADCOPTER_THRUST_HPP

#include <sys/time.h>
#include <map>
#include "ros/ros.h"

#define BATTERY_MAX 5
#define BATTERY_MIN 3

#define BATTERY_LOW 3.0//In V

#define THRUST_GLOBAL_MAX 50000
#define THRUST_GLOBAL_MIN 10001
#define THRUST_OFF 0
#define THRUST_OFFSET_LOW 32000

#define QUADCOPTER_THRUST_RANGE 18000


class QuadcopterThrust
{	
	public:
		QuadcopterThrust();

		/**
		 *
		 */
		void init();

		/**
		 *
		 * @return
		 */
		bool initDone();

		/**
		 *
		 * @param battery
		 * @return
		 */
		bool checkAndSetBatteryValue( float battery );	

		/**
		 *
		 */
		void setWithoutBatteryValue();

		/**
		 *
		 * @param thrust
		 * @return
		 */
		unsigned int checkAndFix( unsigned int thrust );

		/**
		 *
		 * @param thrust
		 * @return
		 */
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

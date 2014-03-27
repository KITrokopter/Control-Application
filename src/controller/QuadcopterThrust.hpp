#ifndef QUADCOPTER_THRUST_HPP
#define QUADCOPTER_THRUST_HPP

#include <sys/time.h>
#include <map>

#define QUADCOPTER_THRUST_RANGE 10000 

class QuadcopterThrust
{	
	public:
		QuadcopterThrust();
		void setThrust( double battery );

		void setMin( unsigned int min );
		unsigned int getMin();
		void setMax( unsigned int max );
		unsigned int getMax();
		void setStartMax( unsigned int startMax );
		unsigned int getStartMax();
		void setStart( unsigned int start );
		unsigned int getStart();

	private:
		/* 
		 * All values should be seen Quadcopter-specific.
		 * Global min/max values will be in Controller.hpp as global variables.
		 */
		unsigned int min;
		unsigned int max;
		unsigned int startMax;
		unsigned int start;

		
		#define THRUST_MAX_START 52000
#define THRUST_MIN 28000
#define THRUST_SHUTDOWN 0
#define THRUST_STAND_STILL 28001
#define THRUST_START 35000
#define THRUST_DECLINE 200
#define THRUST_MAX 48001
#define THRUST_STEP 200

};

#endif // QUADCOPTER_THRUST_HPP

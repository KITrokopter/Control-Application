/*
 * QuadcopterControl.hpp
 *
 *  Created on: 30.03.2014
 *      Author: dwx
 */

#ifndef QUADCOPTERCONTROL_HPP_
#define QUADCOPTERCONTROL_HPP_

#include <sys/time.h>
#include <map>
#include "ros/ros.h"


#define THRUST_GLOBAL_MAX 60000
#define THRUST_GLOBAL_MIN 10001
#define THRUST_OFF 0

#define BATTERY_MAX 8
#define BATTERY_LOW 1

class QuadcopterControl {
public:
	QuadcopterControl();

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
	void setDecline( unsigned int decline );
	unsigned int getDecline();

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

	bool setThrustCalled;
};

#endif /* QUADCOPTERCONTROL_HPP_ */

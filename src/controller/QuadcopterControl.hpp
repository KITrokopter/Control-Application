#ifndef QUADCOPTERCONTROL_HPP_
#define QUADCOPTERCONTROL_HPP_

#include "QuadcopterInfo.hpp"
#include "QuadcopterThrust.hpp"
#include <sys/time.h>
#include <map>
#include "ros/ros.h"

/**
 * Parent class for storing quadcopter-spedific values.
 *
 * @author Dominik Kiefer
 */
class QuadcopterControl {
public:
	QuadcopterControl();

	QuadcopterInfo getInfo();
	void setInfo(QuadcopterInfo newInfo);
	QuadcopterThrust getQuadcopterThrust();
	void setQuadcopterThrust(QuadcopterThrust newQuadcopterThrust);

protected:

private:
	QuadcopterInfo info;
	QuadcopterThrust quadcopterThrust;
};

#endif /* QUADCOPTERCONTROL_HPP_ */

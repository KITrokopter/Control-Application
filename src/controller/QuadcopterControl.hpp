/*
 * QuadcopterControl.hpp
 *
 *  Created on: 30.03.2014
 *      Author: dwx
 */

#ifndef QUADCOPTERCONTROL_HPP_
#define QUADCOPTERCONTROL_HPP_

#include "QuadcopterInfo.hpp"
#include "QuadcopterThrust.hpp"
#include <sys/time.h>
#include <map>
#include "ros/ros.h"


class QuadcopterControl {
	public:
		QuadcopterControl();
		
		QuadcopterInfo getInfo();
		void setInfo( QuadcopterInfo newInfo );
		QuadcopterThrust getQuadcopterThrust();
		void setQuadcopterThrust( QuadcopterThrust newQuadcopterThrust );

	protected:

	private:
		QuadcopterInfo info;
		QuadcopterThrust quadcopterThrust;
		
protected:

private:
};

#endif /* QUADCOPTERCONTROL_HPP_ */

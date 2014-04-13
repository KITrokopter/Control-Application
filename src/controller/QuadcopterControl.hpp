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
		
		/**
		 *
		 * @return
		 */
		QuadcopterInfo getInfo();

		/**
		 *
		 * @param newInfo
		 */
		void setInfo( QuadcopterInfo newInfo );

		/**
		 *
		 * @return
		 */
		QuadcopterThrust getQuadcopterThrust();

		/**
		 *
		 * @param newQuadcopterThrust
		 */
		void setQuadcopterThrust( QuadcopterThrust newQuadcopterThrust );

	protected:

	private:
		QuadcopterInfo info;
		QuadcopterThrust quadcopterThrust;
		
protected:

private:
};

#endif /* QUADCOPTERCONTROL_HPP_ */

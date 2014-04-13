#include <ros/ros.h>

#include "position/IPositionReceiver.hpp"
#include "position/PositionModule.hpp"
#include "controller/Controller.hpp"
#include <stdlib.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "control_application");
	
	// Enable debug level logging.
	if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
		ros::console::notifyLoggerLevelsChanged();
	}
	
	Controller receiver;
	PositionModule p(&receiver);
	
	if (p.isInitialized()) {
		// DO STUFF
		ROS_DEBUG("p.isInitialized true");
		ros::spin();
		for(int i = 0; i < 100; i++)
		{
		  if(ros::ok())
		  {
			  break;
		  }
		  usleep(10000);
		}
	} else {
		ros::shutdown();
	}
		
	while (ros::ok())
	{
		usleep(10000);
	}
	ROS_ERROR("End of main");
}

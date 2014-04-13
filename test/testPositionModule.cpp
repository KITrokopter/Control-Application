#include <ros/ros.h>

#include "helpers/DummyPositionReceiver.hpp"
#include "../src/position/PositionModule.hpp"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "position_module");
	
	// Enable debug level logging.
	if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
		ros::console::notifyLoggerLevelsChanged();
	}
	
	ROS_INFO("Starting...");
	
	IPositionReceiver* receiver = new DummyPositionReceiver();
	PositionModule p(receiver);
	
	if (p.isInitialized()) {
		// DO STUFF
		ros::spin();
	}
	
	delete receiver;
}

#include <ros/ros.h>

#include "position/IPositionReceiver.hpp"
#include "position/DummyPositionReceiver.hpp"
#include "position/PositionModule.hpp"
#include "controller/Controller.hpp"
#include <stdlib.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "position_module");
	IPositionReceiver* receiver = new Controller();
	PositionModule p(receiver);
	
	if (p.isInitialized()) {
		// DO STUFF
		ros::spin();
	} else {
		ros::shutdown();
	}
	
	while (ros::ok())
	{
		usleep(10000);
	}
	
	delete receiver;
}
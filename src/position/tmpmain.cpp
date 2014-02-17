#include <ros/ros.h>

#include "IPositionReceiver.hpp"
#include "DummyPositionReceiver.hpp"
#include "PositionModule.hpp"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "position_module");
	IPositionReceiver* receiver = new DummyPositionReceiver();
	PositionModule p(receiver);
	
	// DO STUFF
	ros::spin();
	
	delete receiver;
}
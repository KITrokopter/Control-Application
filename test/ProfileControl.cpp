#include "helpers/MovementGenerator.hpp"
#include "../src/controller/Controller.hpp"
#include "ros/ros.h"
#include <iostream>
#include "unistd.h"

int main(int argc, char** argv) {
	ros::init(argc, argv, "profile_control_node");
	
	std::cout << "Start profile_control" << std::endl;

	/*
	 * TODO: needs to be initialized
	 * Controller instead of DummyPositionReceiver
	 */
	Controller *receiver = new Controller(); 
	
	std::vector<Vector> from;
	std::vector<Vector> to;
	
	from.push_back(Vector(0, 0, 0));
	to.push_back(Vector(1, 1, 1));
	from.push_back(Vector(0, 0, 1));
	to.push_back(Vector(1, 1, 2));
	from.push_back(Vector(0, 1, 0));
	to.push_back(Vector(1, 2, 1));
	from.push_back(Vector(1, 0, 0));
	to.push_back(Vector(2, 1, 1));
	from.push_back(Vector(1, 1, 1));
	to.push_back(Vector(2, 2, 2));
		
	ROS_INFO("Spinning");
	ros::AsyncSpinner spinner(1); // Use 4 threads
	spinner.start();
	//ros::waitForShutdown();
	//ros::MultiThreadedSpinner spinner(2); // Use 4 threads
	//spinner.spin(); // spin() will not return until the node has been shutdown
	
	usleep(100000000);	
	
	// See MovementGenerator.hpp
	MovementGenerator generator(receiver, from, to, 0.05, 0.1, 0.01, 600, 30);
	ROS_INFO("Generating");
	generator.run();
	spinner.stop();
	ros::waitForShutdown();
}

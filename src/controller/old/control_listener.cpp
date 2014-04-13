void MoveFormationCallback(const control_application::MoveFormation::ConstPtr &msg)
{
	ROS_INFO("I heard: %i", msg->xMovement);
}

int main(int argc, char **argv)
{
// TODO: Create unique name.
	ros::init(argc, argv, "control_listener");

	/**
	 * NodeHandle is the main access point to communications with the ROS
	 * system.
	 * The first NodeHandle constructed will fully initialize this node, and the
	 * last
	 * NodeHandle destructed will close down the node.
	 */
	ros::NodeHandle n;
	// Subscriber for the MoveFormation data of the Quadcopts (1000 is the max.
	// buffered messages)
	ros::Subscriber MoveFormation_sub = n.subscribe("MoveFormation", 1000, moveFormationCallback);

	ros::spin();

/*
 *     ros::shutdown();
 *
 *     // Wait for ros to shutdown.
 *     while (ros::ok()) {
 *         usleep(10000);
 *     }
 */
}


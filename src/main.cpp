
int main(int argc, char** argv)
{
// TODO: Create unique name.
	ros::init(argc, argv, "controller");
	
	/**
  	 * NodeHandle is the main access point to communications with the ROS system.
	 * The first NodeHandle constructed will fully initialize this node, and the last
     * NodeHandle destructed will close down the node.
     */
    ros::NodeHandle n;
	//Publisher for the Movement data of the Quadcopts (1000 is the max. buffered messages)
	ros::Publisher Movement_pub = n.advertise<control_application::Movement>("Movement", 1000);
	
	//100 publishes per second
	ros::Rate loop_rate(100);
  
    while (ros::ok())
     {
       /**
        * This is a message object. You stuff it with data, and then publish it.
        */
       control_application::Movement control_msg;
   
	   //get data from controller::makeMovement() (maybe loop over the amount of crazyflies)
       control_msg.thrust = ;
       control_msg.yaw = ;
       control_msg.pitch = ;
       control_msg.roll = ;
       control_msg.id = ;	
   
       Movement_pub.publish(control_msg);
	   //necessary for subscriber
       ros::spinOnce();
  	   //sleep to secure chosen rate
       loop_rate.sleep();
    }

	ros::shutdown();
	
	// Wait for ros to shutdown.
	while (ros::ok()) {
		usleep(10000);
	}

}

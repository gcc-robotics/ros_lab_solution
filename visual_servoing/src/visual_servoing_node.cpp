#include <iostream>
#include <stdio.h>

#include "ros/ros.h"
#include "visual_servoing.cpp"

int main(int argc, char **argv)
{
	// Start Node
	ROS_INFO("Starting visual_servoing_node");

	ros::init(argc, argv, "visual_servoing");
	ros::NodeHandle node;

	// Start trash detector
	visual_servoing c = visual_servoing(node);
	ROS_INFO("Press Ctrl-C to kill node.");

	// Spin
	// ros::Rate loopRate(10); // 10 hz

	// while(ros::ok())
	// {
	// 	// Do other things?

	// 	// ROS Spin & Sleep
	// 	ros::spinOnce();
	// 	loopRate.sleep();
	// }

	ros::spin();

	ros::shutdown();

	return 0;
}

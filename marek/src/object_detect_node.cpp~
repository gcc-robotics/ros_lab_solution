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

	// Start visual servoing
	visual_servoing vs = visual_servoing(node);
	ROS_INFO("Press Ctrl-C to kill node.");

	ros::spin();

	ros::shutdown();

	return 0;
}

#include <iostream>
#include <stdio.h>

#include "ros/ros.h"
#include "object_detect.cpp"

int main(int argc, char **argv)
{
	// Start Node
	ROS_INFO("Starting object_detect_node");

	ros::init(argc, argv, "object_detect");
	ros::NodeHandle node;

	// Start visual servoing
	object_detect ob = object_detect(node);
	ROS_INFO("Press Ctrl-C to kill node.");

	ros::spin();

	ros::shutdown();

	return 0;
}

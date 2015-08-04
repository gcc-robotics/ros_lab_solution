#include <iostream>
#include <stdio.h>
#include <string.h>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "IrobotCalibrate.cpp"

int main(int argc, char **argv)
{
	// Start Node
	ROS_INFO("Starting calibration...");

	ros::init(argc, argv, "irobot_ir_calibrate");
	ros::NodeHandle node;
	IrobotCalibrate calibrate = IrobotCalibrate(node);

	// Spin
	ros::Rate loopRate(calibrate.getRate());

	while(ros::ok() && !calibrate.done())
	{
		calibrate.spinOnce();
		
		ros::spinOnce();
		loopRate.sleep();
	}

	ros::shutdown();

	calibrate.generateGraphs();

	return 0;
}

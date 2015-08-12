#include <iostream>
#include <stdio.h>
#include <string.h>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "IrobotCalibrate.cpp"

// Install: sudo apt-get install mathgl libmgl-dev
//
// Run:		hcitool scan
//			sudo rfcomm connect 0 ADDRESS 1
//			roslaunch turtlebot_bringup minimal.launch
//			rosrun irobot_ir_calibrate
//
// Issues:	sudo rm /dev/rfcomm0
//			rfcomm release 0

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
	
	ROS_INFO("Generating Graph PNG...");
	calibrate.generateGraphs();
	
	ROS_INFO("Done!");
	ros::shutdown();

	return 0;
}

#include "ros/ros.h"
#include <cmath>
#include "geometry_msgs/Twist.h"
#include <unistd.h>
#include "create_node/TurtlebotSensorState.h"
#include <iostream>
#include <stdio.h>

#ifndef __IROBOT_CALIBRATE_H_
#define __IROBOT_CALIBRATE_H_


class IrobotCalibrate
{
	private:
		// ROS Specific
		ros::NodeHandle node;
		ros::Subscriber irSubscriber;
		ros::Publisher velocityPublisher;
		geometry_msgs::Twist velocityCommand;

		// Timing
		int rate;
		double secondsPerDirection;
		int totalIterations;
		int currentIteration;
		bool dataUpdated;

		// Speeds
		double moveSpeed;
		
		// Current sensor data
		int currentLeftIrSensor;
		int currentFarLeftIrSensor;
		int currentRightIrSensor;
		int currentFarRightIrSensor;

		// Data History
		int* leftIrSensorData;
		int* farLeftIrSensorData;
		int* rightIrSensorData;
		int* farRightIrSensorData;

	public:
		IrobotCalibrate(ros::NodeHandle rosNode);
		int getRate();
		bool done();
		void spinOnce();
		void storeCurrentData();
		void generateGraphs();

		void irCallback(const create_node::TurtlebotSensorState::ConstPtr& msg);
};

#endif

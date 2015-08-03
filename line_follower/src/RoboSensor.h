#include "ros/ros.h"
#include <cmath>
#include "geometry_msgs/Twist.h"
#include <unistd.h>
#include "create_node/TurtlebotSensorState.h"

#ifndef __RoboSensor_H_
#define __RoboSensor_H_


class RoboSensor
{
	private:
		ros::NodeHandle node;
		geometry_msgs::Twist velocityCommand;
		ros::Publisher velocityPublisher;
		geometry_msgs::Twist command;
		ros::Subscriber bumperSubscriber;
		bool rightIR;
		bool leftIR;

	public:
		RoboSensor(ros::NodeHandle rosNode);
		void goForward();
		void goBackward();
		void rotateLeft();
		void rotateRight();
		void checkIR();
		void irCallback(const create_node::TurtlebotSensorState::ConstPtr& msg);
		void goRobotGo();
};

#endif

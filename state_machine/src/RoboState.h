#include "ros/ros.h"
#include <cmath>
#include "geometry_msgs/Twist.h"
#include <unistd.h>
#include "create_node/TurtlebotSensorState.h"

#ifndef __RoboState_H_
#define __RoboState_H_


class RoboState
{
	private:
		ros::NodeHandle node;
		geometry_msgs::Twist velocityCommand;
		ros::Publisher velocityPublisher;
		geometry_msgs::Twist command;
		ros::Subscriber bumperSubscriber;
		bool rightBumperState;
		bool leftBumperState;

	public:
		RoboState(ros::NodeHandle rosNode);
		void goForward();
		void checkWall();
		void goBackward();
		void rotateLeft();
		void rotateRight();
		void checkBumper();
		void bumperCallback(const create_node::TurtlebotSensorState::ConstPtr& msg);
		void goRobotGo();
};

#endif

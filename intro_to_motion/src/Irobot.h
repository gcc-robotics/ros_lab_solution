#include "ros/ros.h"
#include <cmath>
#include "geometry_msgs/Twist.h"
#include <unistd.h>


#ifndef __Irobot_H_
#define __Irobot_H_


class Irobot
{
	private:
		ros::NodeHandle node;
		geometry_msgs::Twist velocityCommand;
		ros::Publisher velocityPublisher;
		geometry_msgs::Twist command;

	public:
		Irobot(ros::NodeHandle rosNode);
		void sendCommand();
		void setLinearX(float);
		void setAngularZ(float);
		void goRobotGo();
};

#endif

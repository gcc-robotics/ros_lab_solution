#include "RoboSensor.h"


RoboSensor::RoboSensor(ros::NodeHandle rosNode)
{
	this->node = rosNode;
	this->velocityPublisher = this->node.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
	this->irSubscriber = this->node.subscribe("/mobile_base/sensors/core", 1, &RoboSensor::irCallback, this);
	counter = 0;
}

void RoboSensor::goForward()
{
	this->velocityCommand.linear.x = 1.0;
	this->velocityCommand.angular.z = 0.0;

	velocityPublisher.publish(this->velocityCommand);
}

void RoboSensor::goBackward()
{
	this->velocityCommand.linear.x = -1.0;
	this->velocityCommand.angular.z = 0.0;

	velocityPublisher.publish(this->velocityCommand);
}

void RoboSensor::rotateLeft()
{
	this->velocityCommand.linear.x = 0.0;
	this->velocityCommand.angular.z = -1.0;

	velocityPublisher.publish(this->velocityCommand);
}

void RoboSensor::rotateRight()
{
	this->velocityCommand.linear.x = 0.0;
	this->velocityCommand.angular.z = 1.0;

	velocityPublisher.publish(this->velocityCommand);
}

void RoboSensor::checkIrSensor()
{
	if(farLeftIrSensor < 500)
	{
		rotateLeft();
	}
	else if(leftIrSensor < 500)
	{
		rotateLeft();
	}
	else if(farRightIrSensor < 500)
	{
		rotateRight();
	}
	else if(rightIrSensor < 500)
	{
		rotateRight();
	}
	else
	{
		goForward();
	}
}

void RoboSensor::checkForLap()
{
	if(farRightIrSensor > 1700 || farLeftIrSensor > 1700 || rightIrSensor > 1700 || leftIrSensor > 1700)
	{
		counter++;
	}
}

void RoboSensor::finishLoop()
{
	if(counter == 5)
	{
		if(farLeftIrSensor > 1700)
		{
			rotateLeft();
		}
		else if(leftIrSensor > 1700)
		{
			rotateLeft();
		}
		else if(farRightIrSensor > 1700)
		{
			rotateRight();
		}
		else if(rightIrSensor > 1700)
		{
			rotateRight();
		}
		else
		{
			goForward();
		}
	}
}

void RoboSensor::irCallback(const create_node::TurtlebotSensorState::ConstPtr& msg)
{
	farLeftIrSensor = msg->cliff_left_signal;
	farRightIrSensor = msg->cliff_right_signal;
	leftIrSensor = msg->cliff_front_left_signal;
	rightIrSensor = msg->cliff_front_right_signal;
}

void RoboSensor::goRobotGo()
{
	checkIrSensor();
	checkForLap();
	finishLoop();
}
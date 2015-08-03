#include "RoboSensor.h"


RoboSensor::RoboSensor(ros::NodeHandle rosNode)
{
	this->node = rosNode;
	this->velocityPublisher = this->node.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
	this->bumperSubscriber = this->node.subscribe("/mobile_base/sensors/core", 1, &RoboSensor::bumperCallback, this);
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

void RoboSensor::goRobotGo()
{
	goForward();
	rotateRight();
	goForward();
	rotateRight();
	goForward();
	rotateRight();
	goForward();
	rotateRight();
}
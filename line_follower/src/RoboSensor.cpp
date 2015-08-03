#include "RoboSensor.h"


RoboSensor::RoboSensor(ros::NodeHandle rosNode)
{
	this->node = rosNode;
	this->velocityPublisher = this->node.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
	this->irSubscriber = this->node.subscribe("/mobile_base/sensors/core", 1, &RoboSensor::irCallback, this);
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
	if(farLeftIrSensor < 100 && farLeftIrSensor > 50)
	{
		rotateLeft();
	}
	else if(leftIrSensor < 100 && leftIrSensor > 50)
	{
		rotateLeft();
	}
	else if(farRightIrSensor < 100 && farRightIrSensor > 50)
	{
		rotateRight();
	}
	else if(rightIrSensor < 100 && rightIrSensor > 50)
	{
		rotateRight();
	}
	else
	{
		goForward();
	}
}

void RoboSensor::irCallback(const create_node::TurtlebotSensorState::ConstPtr& msg)
{
	farLeftIrSensor = msg->IRSENSOR;
	farRightIrSensor = msg->IRSENSOR;
	leftIrSensor = msg->IRSENSOR;
	rightIrSensor = msg->IRSENSOR;
}

void RoboSensor::goRobotGo()
{
	checkIrSensor();
}
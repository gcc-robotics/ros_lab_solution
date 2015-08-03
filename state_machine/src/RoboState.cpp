#include "RoboState.h"


RoboState::RoboState(ros::NodeHandle rosNode)
{
	this->node = rosNode;
	this->velocityPublisher = this->node.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
	this->bumperSubscriber = this->node.subscribe("/mobile_base/sensors/core", 1, &RoboState::bumperCallback, this);
}

void RoboState::goForward()
{
	this->velocityCommand.linear.x = 1.0;
	this->velocityCommand.angular.z = 0.0;

	velocityPublisher.publish(this->velocityCommand);
}

void RoboState::checkWall()
{
	this->velocityCommand.linear.x = 1.0;
	this->velocityCommand.angular.z = 0.25;

	velocityPublisher.publish(this->velocityCommand);
}

void RoboState::goBackward()
{
	this->velocityCommand.linear.x = -1.0;
	this->velocityCommand.angular.z = 0.0;

	velocityPublisher.publish(this->velocityCommand);
}

void RoboState::rotateLeft()
{
	this->velocityCommand.linear.x = 0.0;
	this->velocityCommand.angular.z = -1.0;

	velocityPublisher.publish(this->velocityCommand);
}

void RoboState::rotateRight()
{
	this->velocityCommand.linear.x = 0.0;
	this->velocityCommand.angular.z = 1.0;

	velocityPublisher.publish(this->velocityCommand);
}

void RoboState::checkBumper()
{
	if(rightBumperState)
	{
		rotateLeft();
	}
	else if(leftBumperState)
	{
		rotateRight();
	}
	else
	{
		//twidle thumb
	}

}

void RoboState::bumperCallback(const create_node::TurtlebotSensorState::ConstPtr& msg)
{
	if(msg->bumps_wheeldrops == 1)
	{
		rightBumperState = true;
		leftBumperState = false;
	}
	else if(msg->bumps_wheeldrops == 2)
	{
		rightBumperState = false;
		leftBumperState = true;
	}
	else if(msg->bumps_wheeldrops == 3)
	{
		rightBumperState = true;
		leftBumperState = true;
	}
	else
	{
		rightBumperState = false;
		leftBumperState = false;
	}
}

void RoboState::goRobotGo()
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
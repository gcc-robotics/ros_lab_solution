#include "RoboState.h"

/**
	Constructor for Irobot class

	@param	rosNode	ros::NodeHandle
	@return			void
*/
RoboState::RoboState(ros::NodeHandle rosNode)
{
	this->node = rosNode;
	this->velocityPublisher = this->node.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
	this->bumperSubscriber = this->node.subscribe("/mobile_base/sensors/core", 1, RoboState::bumperCallback);
}

void RoboState::goFoward()
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
		turnLeft();
	}
	else if(leftBumperState)
	{
		turnRight();
	}
	else
	{
		//twidle thumb
	}

}

void RoboState::bumperCallback()
{

}
#include <image_transport/image_transport.h>
#include "ros/ros.h"
#include "TrackedObject.cpp"

#ifndef __VISUAL_SERVOING_H_
#define __VISUAL_SERVOING_H_

class visual_servoing
{
	private:
		ros::NodeHandle node;
		ros::Subscriber subscriber;
		cv::Point clickedPoint;
		TrackedObject robot;

	public:
		visual_servoing(ros::NodeHandle rosNode);
		void processImage(const sensor_msgs::ImageConstPtr& msg);
};

#endif

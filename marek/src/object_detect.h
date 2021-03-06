#include <image_transport/image_transport.h>
#include "ros/ros.h"
#include "TrackedObject.cpp"

#ifndef __OBJECT_DETECT_H_
#define __OBJECT_DETECT_H_

class object_detect
{
	private:
		ros::NodeHandle node;
		ros::Subscriber subscriber;
		cv::Point clickedPoint;
		TrackedObject shape;

	public:
		object_detect(ros::NodeHandle rosNode);
		void processImage(const sensor_msgs::ImageConstPtr& msg);
};

#endif

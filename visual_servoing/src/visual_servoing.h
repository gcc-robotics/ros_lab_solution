#include <image_transport/image_transport.h>
#include "ros/ros.h"

#ifndef __visual_servoing_H_
#define __visual_servoing_H_


class visual_servoing
{
private:
	ros::NodeHandle node;
	// image_transport::ImageTransport imageTransport;
	ros::Subscriber subscriber;

	cv::Point clickedPoint;

public:
	visual_servoing(ros::NodeHandle rosNode);
	void processImage(const sensor_msgs::ImageConstPtr& msg);
	// void trackRobot();

};

#endif

#include <image_transport/image_transport.h>
#include "ros/ros.h"
#include "object.cpp"

#ifndef __visual_servoing_H_
#define __visual_servoing_H_

#define windowName "Original Image"
#define windowName2 "Thresholded Image"
#define trackbarWindowName "Trackbars"


class visual_servoing
{
private:
	ros::NodeHandle node;
	ros::Subscriber subscriber;
	cv::Point clickedPoint;
	Object robot;

public:
	visual_servoing(ros::NodeHandle rosNode);
	void processImage(const sensor_msgs::ImageConstPtr& msg);
	// void drawRobot(Object object, Mat &frame);
	// void morphOps(Mat &thresh);
	// void trackRobot(Mat threshold, Mat HSV, Mat &cameraFeed);

};

#endif

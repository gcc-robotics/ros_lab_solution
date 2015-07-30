#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "visual_servoing.h"
#include "object.cpp"
#include "img_process.cpp"

void mouseEventCallback(int event, int x, int y, int flags, void* ptr)
{
	switch(event)
	{
		case CV_EVENT_LBUTTONDOWN:

			cv::Point* p = (cv::Point*) ptr;
			p->x = x;
			p->y = y;

			break;
	}
}

visual_servoing::visual_servoing(ros::NodeHandle rosNode)
{
	this->node = rosNode;
	// Subscribe to the camera feed and advertise trash locations
	this->subscriber = this->node.subscribe("/camera/rgb/image_raw", 1, &visual_servoing::processImage, this);

	this->clickedPoint = cv::Point();
}

void visual_servoing::processImage(const sensor_msgs::ImageConstPtr& msg)
{
	// Convert the ROS image to an OpenCV Image
	cv_bridge::CvImagePtr cv_ptr;

	try
	{
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	}
	catch(cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	Mat camerafeed = cv_ptr->image;
	Mat threshold;
	Mat HSV;

	// visual_servoing the threshed image
	createTrackbars();
	cvtColor(cv_ptr->image,HSV,COLOR_BGR2HSV);

	cvtColor(cv_ptr->image,HSV,COLOR_BGR2HSV);
	inRange(HSV,Scalar(H_MIN,S_MIN,V_MIN),Scalar(H_MAX,S_MAX,V_MAX), threshold);
	morphOps(threshold);
	imshow(windowName2,threshold);
	trackRobot(threshold,HSV,cv_ptr->image);
	cv::setMouseCallback("Original Image", mouseEventCallback, &(this->clickedPoint));
	
	// Draw target point
	if(this->clickedPoint.x != -1 && this->clickedPoint.y != -1)
	{
		cv::circle(cv_ptr->image, cv::Point(this->clickedPoint.x, this->clickedPoint.y), 5, cv::Scalar(255, 0, 0));
	}
	
	imshow(windowName,cv_ptr->image);
	waitKey(3);

}

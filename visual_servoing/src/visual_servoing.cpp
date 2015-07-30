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

/**
	Function that takes in vector of Object objects and openCV Mat image

	@param	object	std::vector<Object>
	@param	&frame	cv::Mat
	@return	void
*/
void drawRobot(Object object, Mat &frame)
{
	int x = object.getXPos();
	int y = object.getYPos();

	cv::circle(frame, cv::Point(x, y),5, cv::Scalar(0, 0, 255));
	cv::circle(frame, cv::Point(x, y), 40, cv::Scalar(0, 255, 0));
	cv::putText(frame, intToString(x)+ " , " 
				+ intToString(y), cv::Point(x, y + 20), 1, 1,Scalar(0, 255, 0));
	cv::putText(frame, object.getType(), cv::Point(x, y - 30)
				, 1 , 2, object.getColor());
}

/**
	Function to erode and dilate whitespace

	@param	&thresh	cv::Mat
	@return	void
*/
void morphOps(Mat &thresh)
{
	Mat erodeElement = getStructuringElement( MORPH_RECT,Size(4,4));
	Mat dilateElement = getStructuringElement( MORPH_RECT,Size(10,10));

	erode(thresh,thresh,erodeElement);
	erode(thresh,thresh,erodeElement);

	dilate(thresh,thresh,dilateElement);
	dilate(thresh,thresh,dilateElement);
}

/**
	Function to be used with calibrate node for tracking objects

	@param	threshold	cv::Mat
	@param	HSV			cv::Mat
	@param	&cameraFeed	cv::Mat
	@return	void
*/
void trackRobot(Object robot, Mat threshold, Mat HSV, Mat &cameraFeed)
{
	cv::Mat temp;
	threshold.copyTo(temp);
	std::vector< std::vector<Point> > contours;
	std::vector<Vec4i> hierarchy;
	cv::findContours(temp, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

	double refArea = 0;
	bool objectFound = false;
	if (hierarchy.size() > 0)
	{
		for (int index = 0; index >= 0; index = hierarchy[index][0])
		{
			Moments moment = moments((cv::Mat)contours[index]);
			double area = moment.m00;

			if(area > MIN_OBJECT_AREA)
			{

				robot.setXPos(moment.m10/area);
				robot.setYPos(moment.m01/area);
			
				objectFound = true;

			}
			else
			{
				objectFound = false;
			}
		}
	
		if(objectFound ==true)
		{
			
			drawRobot(robot,cameraFeed);
		}

		else
		{
			putText(cameraFeed,"TOO MUCH NOISE! ADJUST FILTER", Point(0, 50), 1, 1, Scalar(0, 0, 255), 2);
		}
	}
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
	trackRobot(this->robot, threshold,HSV,cv_ptr->image);
	cv::setMouseCallback("Original Image", mouseEventCallback, &(this->clickedPoint));

	// Draw target point
	if(this->clickedPoint.x != -1 && this->clickedPoint.y != -1)
	{
		cv::circle(cv_ptr->image, cv::Point(this->clickedPoint.x, this->clickedPoint.y), 5, cv::Scalar(255, 0, 0));
	}
	
	imshow(windowName,cv_ptr->image);
	waitKey(3);

}
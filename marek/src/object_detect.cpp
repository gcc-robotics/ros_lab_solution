#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "object_detect.h"

#define windowName "Original Image"
#define windowName2 "Thresholded Image"
#define trackbarWindowName "Trackbars"

/**
	Function gets called whenever a trackbar position 
	is changed

	@return	void
*/
void on_trackbar( int, void* )
{
	
}

/**
	Function converts integer to string

	@param	number	int
	@return			std::string
*/
std::string intToString(int number)
{


	std::stringstream ss;
	ss << number;
	return ss.str();
}
/**
	Function to create trackbars

	@return	void
*/
void createTrackbars()
{
	namedWindow(trackbarWindowName,0);
	char TrackbarName[50];

	sprintf( TrackbarName, "H_MIN", H_MIN);
	sprintf( TrackbarName, "H_MAX", H_MAX);
	sprintf( TrackbarName, "S_MIN", S_MIN);
	sprintf( TrackbarName, "S_MAX", S_MAX);
	sprintf( TrackbarName, "V_MIN", V_MIN);
	sprintf( TrackbarName, "V_MAX", V_MAX);
   
	createTrackbar( "H_MIN", trackbarWindowName, &H_MIN, H_MAX, on_trackbar );
	createTrackbar( "H_MAX", trackbarWindowName, &H_MAX, H_MAX, on_trackbar );
	createTrackbar( "S_MIN", trackbarWindowName, &S_MIN, S_MAX, on_trackbar );
	createTrackbar( "S_MAX", trackbarWindowName, &S_MAX, S_MAX, on_trackbar );
	createTrackbar( "V_MIN", trackbarWindowName, &V_MIN, V_MAX, on_trackbar );
	createTrackbar( "V_MAX", trackbarWindowName, &V_MAX, V_MAX, on_trackbar );
}


object_detect::object_detect(ros::NodeHandle rosNode)
{
	this->node = rosNode;
	// Subscribe to the camera feed 
	this->subscriber = this->node.subscribe("/camera/rgb/image_raw", 1, &object_detect::processImage, this);

}

/**
	Function that takes in vector of TrackedObject objects and openCV Mat image

	@param	object	std::vector<TrackedObject>
	@param	&frame	cv::Mat
	@return	void
*/
void drawShape(TrackedObject shape, Mat &frame)
{
	int x = shape.getXPos();
	int y = shape.getYPos();

	cv::circle(frame, cv::Point(x, y),5, cv::Scalar(0, 0, 255));
	cv::circle(frame, cv::Point(x, y), 40, cv::Scalar(0, 255, 0));
	cv::putText(frame, intToString(x)+ " , " 
				+ intToString(y), cv::Point(x, y + 20), 1, 1,Scalar(0, 255, 0));
	cv::putText(frame, shape.getType(), cv::Point(x, y - 30)
				, 1 , 2, shape.getColor());
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
void trackShape(TrackedObject& shape, Mat threshold, Mat HSV, Mat &cameraFeed)
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
				shape.setXPos(moment.m10/area);
				shape.setYPos(moment.m01/area);
			
				objectFound = true;
			}
			else
			{
				objectFound = false;
			}
		}
	
		if(objectFound ==true)
		{
			drawShape(shape,cameraFeed);
		}
		else
		{
			putText(cameraFeed,"TOO MUCH NOISE! ADJUST FILTER", Point(0, 50), 1, 1, Scalar(0, 0, 255), 2);
		}
	}
}



void object_detect::processImage(const sensor_msgs::ImageConstPtr& msg)
{
	// Convert the ROS image to an OpenCV Image
	cv_bridge::CvImagePtr cv_ptr;

	cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

	Mat camerafeed = cv_ptr->image;
	Mat threshold;
	Mat HSV;

	// object_detect the threshed image
	createTrackbars();
	cvtColor(cv_ptr->image,HSV,COLOR_BGR2HSV);

	cvtColor(cv_ptr->image,HSV,COLOR_BGR2HSV);
	inRange(HSV,Scalar(H_MIN,S_MIN,V_MIN),Scalar(H_MAX,S_MAX,V_MAX), threshold);
	morphOps(threshold);
	imshow(windowName2,threshold);
	trackShape(this->shape, threshold,HSV,cv_ptr->image);

	imshow(windowName,cv_ptr->image);
	waitKey(3);
}

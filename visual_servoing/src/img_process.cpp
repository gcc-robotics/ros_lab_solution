#include "ros/ros.h"

// /**
// 	Function gets called whenever a trackbar position 
// 	is changed

// 	@return	void
// */
// void on_trackbar( int, void* )
// {
	
// }

// /**
// 	Function converts integer to string

// 	@param	number	int
// 	@return			std::string
// */
// std::string intToString(int number)
// {


// 	std::stringstream ss;
// 	ss << number;
// 	return ss.str();
// }
// /**
// 	Function to create trackbars

// 	@return	void
// */
// void createTrackbars()
// {
	
// 	namedWindow(trackbarWindowName,0);
// 	char TrackbarName[50];

// 	sprintf( TrackbarName, "H_MIN", H_MIN);
// 	sprintf( TrackbarName, "H_MAX", H_MAX);
// 	sprintf( TrackbarName, "S_MIN", S_MIN);
// 	sprintf( TrackbarName, "S_MAX", S_MAX);
// 	sprintf( TrackbarName, "V_MIN", V_MIN);
// 	sprintf( TrackbarName, "V_MAX", V_MAX);
// 	sprintf( TrackbarName, "R_MIN", R_MIN);
// 	sprintf( TrackbarName, "R_MAX", R_MAX);
// 	sprintf( TrackbarName, "G_MIN", G_MIN);
// 	sprintf( TrackbarName, "G_MAX", G_MAX);
// 	sprintf( TrackbarName, "B_MIN", B_MIN);
// 	sprintf( TrackbarName, "B_MAX", B_MAX);
   
// 	createTrackbar( "H_MIN", trackbarWindowName, &H_MIN, H_MAX, on_trackbar );
// 	createTrackbar( "H_MAX", trackbarWindowName, &H_MAX, H_MAX, on_trackbar );
// 	createTrackbar( "S_MIN", trackbarWindowName, &S_MIN, S_MAX, on_trackbar );
// 	createTrackbar( "S_MAX", trackbarWindowName, &S_MAX, S_MAX, on_trackbar );
// 	createTrackbar( "V_MIN", trackbarWindowName, &V_MIN, V_MAX, on_trackbar );
// 	createTrackbar( "V_MAX", trackbarWindowName, &V_MAX, V_MAX, on_trackbar );
// 	createTrackbar( "R_MIN", trackbarWindowName, &R_MIN, R_MAX, on_trackbar );
// 	createTrackbar( "R_MAX", trackbarWindowName, &R_MAX, R_MAX, on_trackbar );
// 	createTrackbar( "G_MIN", trackbarWindowName, &G_MIN, G_MAX, on_trackbar );
// 	createTrackbar( "G_MAX", trackbarWindowName, &G_MAX, G_MAX, on_trackbar );
// 	createTrackbar( "B_MIN", trackbarWindowName, &B_MIN, B_MAX, on_trackbar );
// 	createTrackbar( "B_MAX", trackbarWindowName, &B_MAX, B_MAX, on_trackbar );


// }

// /**
// 	Function that takes in vector of Object objects and openCV Mat image

// 	@param	object	std::vector<Object>
// 	@param	&frame	cv::Mat
// 	@return	void
// */
// void drawRobot(Object object, Mat &frame)
// {
// 	int x = object.getXPos();
// 	int y = object.getYPos();

// 	cv::circle(frame, cv::Point(x, y),5, cv::Scalar(0, 0, 255));
// 	cv::circle(frame, cv::Point(x, y), 40, cv::Scalar(0, 255, 0));
// 	cv::putText(frame, intToString(x)+ " , " 
// 				+ intToString(y), cv::Point(x, y + 20), 1, 1,Scalar(0, 255, 0));
// 	cv::putText(frame, object.getType(), cv::Point(x, y - 30)
// 				, 1 , 2, object.getColor());
// }

// /**
// 	Function to erode and dilate whitespace

// 	@param	&thresh	cv::Mat
// 	@return	void
// */
// void morphOps(Mat &thresh)
// {
// 	Mat erodeElement = getStructuringElement( MORPH_RECT,Size(4,4));
// 	Mat dilateElement = getStructuringElement( MORPH_RECT,Size(10,10));

// 	erode(thresh,thresh,erodeElement);
// 	erode(thresh,thresh,erodeElement);

// 	dilate(thresh,thresh,dilateElement);
// 	dilate(thresh,thresh,dilateElement);
// }

// *
// 	Function to be used with calibrate node for tracking objects

// 	@param	threshold	cv::Mat
// 	@param	HSV			cv::Mat
// 	@param	&cameraFeed	cv::Mat
// 	@return	void

// void trackRobot(Mat threshold, Mat HSV, Mat &cameraFeed)
// {

// 	Object robot;

// 	Mat temp;
// 	threshold.copyTo(temp);
// 	std::vector< std::vector<Point> > contours;
// 	std::vector<Vec4i> hierarchy;
// 	findContours(temp, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

// 	double refArea = 0;
// 	bool objectFound = false;
// 	if (hierarchy.size() > 0)
// 	{
// 		int numObjects = hierarchy.size();

// 		if(numObjects < MAX_NUM_OBJECTS)
// 		{
// 			for (int index = 0; index >= 0; index = hierarchy[index][0])
// 			{
// 				Moments moment = moments((cv::Mat)contours[index]);
// 				double area = moment.m00;

// 				if(area > MIN_OBJECT_AREA)
// 				{

// 					robot.setXPos(moment.m10/area);
// 					robot.setYPos(moment.m01/area);
				
// 					objectFound = true;

// 				}
// 				else
// 				{
// 					objectFound = false;
// 				}
// 			}
		
// 			if(objectFound ==true)
// 			{
				
// 				drawRobot(robot,cameraFeed);
// 			}

// 		}
// 		else
// 		{
// 			putText(cameraFeed,"TOO MUCH NOISE! ADJUST FILTER", Point(0, 50), 1, 2, Scalar(0, 0, 255), 2);
// 		}
// 	}
// }


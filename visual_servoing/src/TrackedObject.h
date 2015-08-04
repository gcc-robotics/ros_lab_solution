#include <iostream>
#include <stdio.h>
#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sstream>
#include <vector>
#include <string>
#include <cmath>
#include "opencv2/core/core.hpp"
#include <opencv/highgui.h>
#include <opencv2/opencv.hpp>
#include <opencv/cv.h>

#ifndef __TRACKED_OBJECT_H_
#define __TRACKED_OBJECT_H_

using namespace cv;

int H_MIN = 0;
int H_MAX = 256;
int S_MIN = 0;
int S_MAX = 256;
int V_MIN = 0;
int V_MAX = 256;
//default capture width and height
const int FRAME_WIDTH = 1280;
const int FRAME_HEIGHT = 960;
//minimum and maximum object area
const int MIN_OBJECT_AREA = 30 * 30;
const int MAX_OBJECT_AREA = FRAME_HEIGHT*FRAME_WIDTH / 1.5;

class TrackedObject
{
	public:
		TrackedObject();
		TrackedObject(std::string name);
		void setXPos(int x);
		int getXPos();
		void setYPos(int y);
		int getYPos();
		Scalar getHSVmin();
		Scalar getHSVmax();
		void setHSVmin(Scalar min);
		void setHSVmax(Scalar max);
		std::string getType();
		void setType(std::string t);
		Scalar getColor();
		void setColor(Scalar c);
		double getDistance();
		double getAngle();
	private:
		int xPos;
		int yPos;
		std::string type;
		Scalar HSVmin, HSVmax;
		Scalar Color;
	};


#endif
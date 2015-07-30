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
using namespace cv;


int H_MIN = 0;
int H_MAX = 256;
int S_MIN = 0;
int S_MAX = 256;
int V_MIN = 0;
int V_MAX = 256;
int R_MIN = 0;
int R_MAX = 255;
int G_MIN = 0;
int G_MAX = 255;
int B_MIN = 0;
int B_MAX = 255;
//default capture width and height
const int FRAME_WIDTH = 1280;
const int FRAME_HEIGHT = 960;
//max number of objects to be detected in frame
const int MAX_NUM_OBJECTS = 50;
//minimum and maximum object area
const int MIN_OBJECT_AREA = 30 * 30;
const int MAX_OBJECT_AREA = FRAME_HEIGHT*FRAME_WIDTH / 1.5;
const double areaLeft = FRAME_WIDTH / 3.0;		
const double areaRight = 2.0 * FRAME_WIDTH / 3.0;

class Object
{
public:
	Object();
	Object(std::string name);
	void setXPos(int x);
	int getXPos();
	void setYPos(int y);
	int getYPos();
	Scalar getHSVmin();
	Scalar getHSVmax();
	Scalar getRGBmin();
	Scalar getRGBmax();
	void setHSVmin(Scalar min);
	void setHSVmax(Scalar max);
	void setRGBmin(Scalar min);
	void setRGBmax(Scalar max);
	std::string getType(){return type;}
	void setType(std::string t){type = t;}
	Scalar getColor(){
		return Color;
	}
	void setColor(Scalar c){
		Color = c;
	}
	double getDistance();
	double getAngle();
private:
	int xPos;
	int yPos;
	std::string type;
	Scalar HSVmin, HSVmax, RGBmin, RGBmax;
	Scalar Color;
};



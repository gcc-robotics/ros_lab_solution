#include "object.h"

/**
Default Constructor for Object class
*/
Object::Object()
{
	xPos = 0;
	yPos = 0;
	type = "";
	setType("robot position");
	setColor(Scalar(0,0,0));
}

Object::Object(std::string name)
{
	setType(name);
	
	if(name=="blue object detected")
	{
		setHSVmin(Scalar(46,83,24));
		setHSVmax(Scalar(118,236,152));
		setColor(Scalar(255,255,0));

	}
	if(name=="red object detected")
	{
		setHSVmin(Scalar(0,192,0));
		setHSVmax(Scalar(256,256,256));
		setColor(Scalar(0,0,255));
	}
}

void Object::setXPos(int x)
{
	xPos = x;
}

int Object::getXPos()
{
	return xPos;	
}

void Object::setYPos(int y)
{
	yPos = y;
}

int Object::getYPos()
{
	return yPos;	
}

Scalar Object::getHSVmin()
{
	return HSVmin;
}

Scalar Object::getHSVmax()
{
	return HSVmax;
}

Scalar Object::getRGBmin()
{
	return RGBmin;
}

Scalar Object::getRGBmax()
{
	return RGBmax;
}

void Object::setHSVmin(Scalar min)
{
	HSVmin = min;
}

void Object::setHSVmax(Scalar max)
{
	HSVmax = max;
}

void Object::setRGBmin(Scalar min)
{
	RGBmin = min;
}

void Object::setRGBmax(Scalar max)
{
	RGBmax = max;
}

double Object::getDistance()
{
	double dist;
	dist = sqrt( pow(FRAME_HEIGHT - yPos, 2) + pow(FRAME_WIDTH/2 - xPos, 2));
	//dist = (dist * 150)/(FRAME_WIDTH) ;
	return dist;
}

double Object::getAngle()
{
	double opp = (FRAME_WIDTH/2 - xPos);
	double hyp = sqrt( pow(FRAME_HEIGHT - yPos, 2) + pow(FRAME_WIDTH/2 - xPos, 2));
	double angle = asin(opp/hyp) * (180/M_PI);
	return angle;
}



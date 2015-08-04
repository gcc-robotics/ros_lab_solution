#include "TrackedObject.h"

/**
Default Constructor for TrackedObject class
*/
TrackedObject::TrackedObject()
{
	xPos = 0;
	yPos = 0;
	type = "";
	setType("robot detected");
	setColor(Scalar(0,0,0));
}

TrackedObject::TrackedObject(std::string name)
{
	setType(name);
	
	if(name=="blue TrackedObject detected")
	{
		setHSVmin(Scalar(46,83,24));
		setHSVmax(Scalar(118,236,152));
		setColor(Scalar(255,255,0));

	}
	if(name=="red TrackedObject detected")
	{
		setHSVmin(Scalar(0,192,0));
		setHSVmax(Scalar(256,256,256));
		setColor(Scalar(0,0,255));
	}
}

void TrackedObject::setXPos(int x)
{
	xPos = x;
}

int TrackedObject::getXPos()
{
	return xPos;	
}

void TrackedObject::setYPos(int y)
{
	yPos = y;
}

int TrackedObject::getYPos()
{
	return yPos;	
}

Scalar TrackedObject::getHSVmin()
{
	return HSVmin;
}

Scalar TrackedObject::getHSVmax()
{
	return HSVmax;
}

void TrackedObject::setHSVmin(Scalar min)
{
	HSVmin = min;
}

void TrackedObject::setHSVmax(Scalar max)
{
	HSVmax = max;
}

std::string TrackedObject::getType()
{
	return type;
}
void TrackedObject::setType(std::string t)
{
	type = t;
}

Scalar TrackedObject::getColor()
{
	return Color;
}
void TrackedObject::setColor(Scalar c)
{
	Color = c;
}

double TrackedObject::getDistance()
{
	double dist;
	dist = sqrt( pow(FRAME_HEIGHT - yPos, 2) + pow(FRAME_WIDTH/2 - xPos, 2));
	//dist = (dist * 150)/(FRAME_WIDTH) ;
	return dist;
}

double TrackedObject::getAngle()
{
	double opp = (FRAME_WIDTH/2 - xPos);
	double hyp = sqrt( pow(FRAME_HEIGHT - yPos, 2) + pow(FRAME_WIDTH/2 - xPos, 2));
	double angle = asin(opp/hyp) * (180/M_PI);
	return angle;
}



#include "IrobotCalibrate.h"


IrobotCalibrate::IrobotCalibrate(ros::NodeHandle rosNode)
{
	this->node = rosNode;
	this->velocityPublisher = this->node.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
	this->irSubscriber = this->node.subscribe("/mobile_base/sensors/core", 1, &IrobotCalibrate::irCallback, this);
	
	this->rate = 25;
	this->secondsPerDirection = 2;
	this->totalIterations = this->rate * (this->secondsPerDirection * 2);
	this->currentIteration = 0;
	this->dataUpdated = false;

	this->moveSpeed = 0.15;

	this->leftIrSensorData = new int[this->totalIterations];
	this->farLeftIrSensorData = new int[this->totalIterations];
	this->rightIrSensorData = new int[this->totalIterations];
	this->farRightIrSensorData = new int[this->totalIterations];
}

int IrobotCalibrate::getRate()
{
	return this->rate;
}

bool IrobotCalibrate::done()
{
	return (this->currentIteration >= this->totalIterations);
}

void IrobotCalibrate::spinOnce()
{
	if(!this->done())
	{
		if(this->dataUpdated)
		{
			this->storeCurrentData();

			if(this->currentIteration < (this->totalIterations / 2))
			{
				this->velocityCommand.linear.x = this->moveSpeed;
			}
			else
			{
				this->velocityCommand.linear.x = -this->moveSpeed;
			}

			this->velocityPublisher.publish(this->velocityCommand);

			this->currentIteration++;
		}
	}
}

void IrobotCalibrate::storeCurrentData()
{
	this->leftIrSensorData[this->currentIteration] = this->currentLeftIrSensor;
	this->farLeftIrSensorData[this->currentIteration] = this->currentFarLeftIrSensor;
	this->rightIrSensorData[this->currentIteration] = this->currentRightIrSensor;
	this->farRightIrSensorData[this->currentIteration] = this->currentFarRightIrSensor;
}

void IrobotCalibrate::irCallback(const create_node::TurtlebotSensorState::ConstPtr& msg)
{
	this->currentLeftIrSensor = msg->cliff_front_left_signal;
	this->currentFarLeftIrSensor = msg->cliff_left_signal;
	this->currentRightIrSensor = msg->cliff_front_right_signal;
	this->currentFarRightIrSensor = msg->cliff_right_signal;

	this->dataUpdated = true;
}

void IrobotCalibrate::generateSingleGraph(mglGraph *graph, int* data, int position, char* name)
{
	graph->SubPlot(2, 2, position);
	graph->Title(name);
	
	graph->SetRanges(0, this->totalIterations, 0, 1400);
	graph->SetOrigin(0, 0);
	
	graph->Axis("x");
	graph->Label('x',"Reading #",0);
	
	graph->Axis("y");
	graph->Label('y',"Sensor Value", 0);
	
	mglData test(this->totalIterations);
	
	for(int i = 0; i < this->totalIterations; i++)
	{
		test.a[i] = data[i];
	}
	
	graph->Plot(test);
	graph->Box();
	
}

void IrobotCalibrate::generateGraphs()
{
	mglGraph gr(0, 1200, 800);
	
	this->generateSingleGraph(&gr, this->leftIrSensorData, 0, "Left IR Sensor");
	this->generateSingleGraph(&gr, this->rightIrSensorData, 1, "Right IR Sensor");
	this->generateSingleGraph(&gr, this->farLeftIrSensorData, 2, "Far Left IR Sensor");
	this->generateSingleGraph(&gr, this->farRightIrSensorData, 3, "Far Right IR Sensor");
	
	//sample(&gr);
	
	gr.WritePNG("/home/marekl/test.png", "Marek made this. He is super green!", false);
}
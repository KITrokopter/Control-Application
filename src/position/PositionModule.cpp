#include "PositionModule.hpp"

#include <assert.h>

PositionModule::PositionModule(IPositionReceiver* receiver)
{
	assert(receiver != 0);
	this->receiver = receiver;
	
	isCalibrating = false;
	
	ros::NodeHandle n;
	
	this->startCalibrationService = n.advertiseService("StartCalibration", &PositionModule::startCalibrationCallback, this);
}

PositionModule::~PositionModule()
{
	
}

bool PositionModule::startCalibrationCallback(control_application::StartCalibration::Request &req, control_application::StartCalibration::Response &res)
{
	res.ok = !isCalibrating;
	isCalibrating = true;
	return true;
}
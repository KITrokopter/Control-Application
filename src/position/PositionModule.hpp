#ifndef POSITION_MODULE_HPP
#define POSITION_MODULE_HPP

#include <ros/ros.h>

#include "IPositionReceiver.hpp"
#include "control_application/StartCalibration.h"
#include "control_application/TakeCalibrationPicture.h"
#include "camera_application/PictureSendingActivation.h"

class PositionModule {
private:
	IPositionReceiver* receiver;
	
	// Calibration
	bool isCalibrating;
	
	// ROS network
	ros::ServiceServer startCalibrationService;
	ros::ServiceServer takeCalibrationPictureService;
	
	ros::Publisher pictureSendingActivationPublisher;
	
	// ROS callbacks
	bool startCalibrationCallback(control_application::StartCalibration::Request &req, control_application::StartCalibration::Response &res);
	bool takeCalibrationPictureCallback(control_application::TakeCalibrationPicture::Request &req, control_application::TakeCalibrationPicture::Response &res);
	
public:
	PositionModule(IPositionReceiver* receiver);
	~PositionModule();
};

#endif // POSITION_MODULE_HPP
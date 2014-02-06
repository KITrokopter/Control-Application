#ifndef POSITION_MODULE_HPP
#define POSITION_MODULE_HPP

#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <vector>
#include <inttypes.h>

#include "IPositionReceiver.hpp"
#include "control_application/StartCalibration.h"
#include "control_application/TakeCalibrationPicture.h"
#include "control_application/CalculateCalibration.h"
#include "camera_application/PictureSendingActivation.h"
#include "camera_application/Picture.h"

class PositionModule {
private:
	IPositionReceiver* receiver;
	bool _isInitialized;
	
	// Calibration
	bool isCalibrating;
	std::vector<cv::Mat*> pictureCache;
	std::vector<uint64_t> pictureTimes;
	
	// ROS network
	ros::ServiceServer startCalibrationService;
	ros::ServiceServer takeCalibrationPictureService;
	ros::ServiceServer calculateCalibrationService;
	
	ros::Publisher pictureSendingActivationPublisher;
	ros::Publisher pingPublisher;
	
	ros::Subscriber pictureSubscriber;
	
	int rosId;
	
	// ROS callbacks
	bool startCalibrationCallback(control_application::StartCalibration::Request &req, control_application::StartCalibration::Response &res);
	bool takeCalibrationPictureCallback(control_application::TakeCalibrationPicture::Request &req, control_application::TakeCalibrationPicture::Response &res);
	bool calculateCalibrationCallback(control_application::CalculateCalibration::Request &req, control_application::CalculateCalibration::Response &res);
	
	void pictureCallback(const camera_application::Picture &msg);
	
	// ROS wrappers
	void setPictureSendingActivated(bool activated);
	void sendPing();
	
public:
	PositionModule(IPositionReceiver* receiver);
	~PositionModule();
	bool isInitialized();
};

#endif // POSITION_MODULE_HPP
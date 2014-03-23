#ifndef POSITION_MODULE_HPP
#define POSITION_MODULE_HPP

#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <vector>
#include <map>
#include <inttypes.h>
#include <iostream>
#include <fstream>

#include "IPositionReceiver.hpp"
#include "../KitrokopterMessages.hpp"
#include "control_application/StartCalibration.h"
#include "control_application/TakeCalibrationPicture.h"
#include "control_application/CalculateCalibration.h"
#include "camera_application/PictureSendingActivation.h"
#include "camera_application/Picture.h"
#include "camera_application/RawPosition.h"
#include "camera_application/CameraCalibrationData.h"
#include "api_application/System.h"
#include "../controller/Mutex.hpp"
#include "../matlab/Position.h"
#include "tracking/TrackingWorker.hpp"
#include "IdDictionary.hpp"

class PositionModule {
private:
	bool _isInitialized;
	bool isRunning;
	
	// Calibration
	bool isCalibrating;
	bool isCalibrated;
	int calibrationPictureCount;
	cv::Size boardSize;
	cv::Size realSize;
	Mutex pictureCacheMutex;
	std::map<int, cv::Mat*> pictureCache;
	std::map<int, uint64_t> pictureTimes;
	
	std::map<int, cv::Mat> intrinsicsMatrices;
	std::map<int, cv::Mat> distortionCoefficients;
	std::map<int, std::string> windowNames;
	std::map<int, bool> imageDisplayed;
	
	// Tracking
	TrackingWorker trackingWorker;
	/// Maps network ids to camera numbers for the tracker.
	IdDictionary idDict;
	
	// ROS network
	ros::ServiceServer startCalibrationService;
	ros::ServiceServer takeCalibrationPictureService;
	ros::ServiceServer calculateCalibrationService;
	
	ros::Publisher pictureSendingActivationPublisher;
	ros::Publisher pingPublisher;
	ros::Publisher cameraCalibrationDataPublisher;
	
	ros::Subscriber pictureSubscriber;
	ros::Subscriber systemSubscriber;
	ros::Subscriber rawPositionSubscriber;
	
	int rosId;
	KitrokopterMessages* msg;
	
	// ROS callbacks
	bool startCalibrationCallback(control_application::StartCalibration::Request &req, control_application::StartCalibration::Response &res);
	bool takeCalibrationPictureCallback(control_application::TakeCalibrationPicture::Request &req, control_application::TakeCalibrationPicture::Response &res);
	bool calculateCalibrationCallback(control_application::CalculateCalibration::Request &req, control_application::CalculateCalibration::Response &res);
	
	void pictureCallback(const camera_application::Picture &msg);
	void systemCallback(const api_application::System &msg);
	void rawPositionCallback(const camera_application::RawPosition &msg);
	
	// ROS wrappers
	void setPictureSendingActivated(bool activated);
	void sendPing();
	
	// Logging
	std::ofstream log;
	std::vector<long int> timeLog;
	
	bool calculateCameraNumbers();
	
public:
	PositionModule(IPositionReceiver* receiver);
	~PositionModule();
	bool isInitialized();
};

#endif // POSITION_MODULE_HPP
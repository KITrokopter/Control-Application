#include "PositionModule.hpp"

#include <assert.h>
#include <sys/stat.h>
#include <errno.h>
#include <stdlib.h>
#include <string>
#include <algorithm>
#include <iostream>

#include <ros/console.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "sensor_msgs/Image.h"
#include "api_application/Ping.h"
#include "api_application/Announce.h"

#include "../matlab/Vector.h"
#include "../matlab/profiling.hpp"

PositionModule::PositionModule(IPositionReceiver* receiver) : 
	trackingWorker(receiver)
{
	ROS_DEBUG("Initializing PositionModule");
	
	_isInitialized = true;
	isCalibrating = false;
	isCalibrated = false;
	isRunning = false;
	
	ros::NodeHandle n;
	
	this->pictureSendingActivationPublisher = n.advertise<camera_application::PictureSendingActivation>("PictureSendingActivation", 4);
	this->pingPublisher = n.advertise<api_application::Ping>("Ping", 4);
	this->cameraCalibrationDataPublisher = n.advertise<camera_application::CameraCalibrationData>("CameraCalibrationData", 10);
	
	this->pictureSubscriber = n.subscribe("Picture", 12, &PositionModule::pictureCallback, this);
	this->systemSubscriber = n.subscribe("System", 4, &PositionModule::systemCallback, this);
	this->rawPositionSubscriber = n.subscribe("RawPosition", 32, &PositionModule::rawPositionCallback, this);
	
	this->startCalibrationService = n.advertiseService("StartCalibration", &PositionModule::startCalibrationCallback, this);
	this->takeCalibrationPictureService = n.advertiseService("TakeCalibrationPicture", &PositionModule::takeCalibrationPictureCallback, this);
	this->calculateCalibrationService = n.advertiseService("CalculateCalibration", &PositionModule::calculateCalibrationCallback, this);
	
	// Advertise myself to API
	ros::ServiceClient announceClient = n.serviceClient<api_application::Announce>("announce");
	
	api_application::Announce announce;
	announce.request.type = 3; // 3 means position module
	
	if (announceClient.call(announce))
	{
		rosId = announce.response.id;
		
		if (rosId == ~0 /* -1 */)
		{
			ROS_ERROR("Error! Got id -1");
			_isInitialized = false;
		}
		else
		{
			ROS_INFO("Position module successfully announced. Got id %d", rosId);
		}
	}
	else
	{
		ROS_ERROR("Error! Could not announce myself to API!");
		_isInitialized = false;
	}
	
	msg = new KitrokopterMessages(rosId);
	
	if (_isInitialized)
	{
		ROS_DEBUG("PositionModule initialized");
	} else {
		ROS_ERROR("Could not initialize PositionModule!");
	}
	
	log.open("position_module.log");
	
	if (!log.is_open()) {
		ROS_ERROR("Could not open log file!");
	}
}

PositionModule::~PositionModule()
{
	msg->~KitrokopterMessages();
	log.close();
	
	// TODO: Free picture cache.
	
	ROS_DEBUG("PositionModule destroyed");
}

// Service
bool PositionModule::startCalibrationCallback(control_application::StartCalibration::Request &req, control_application::StartCalibration::Response &res)
{
	res.ok = !isCalibrating;
	
	if (!isCalibrating)
	{
		ROS_INFO("Starting multi camera calibration process");
		
		isCalibrated = false;
		
		// Delete old calibration data.
		system("rm -rf /tmp/calibrationResult/*");
		system("rm -rf /tmp/calibrationImages/*");
		setPictureSendingActivated(true);
		calibrationPictureCount = 0;
		boardSize = cv::Size(req.chessboardWidth, req.chessboardHeight);
		realSize = cv::Size(req.chessboardRealWidth, req.chessboardRealHeight);
	}
	
	isCalibrating = true;
	return true;
}

// Service
bool PositionModule::takeCalibrationPictureCallback(control_application::TakeCalibrationPicture::Request &req, control_application::TakeCalibrationPicture::Response &res)
{
	ROS_DEBUG("Taking calibration picture. Have %d cameras.", idDict.size());
	
	if (!idDict.isTranslated()) {
		idDict.translateIds();
	}
	
	pictureCacheMutex.lock();
	
	std::map<int, cv::Mat*> pictureMap;
	std::map<int, bool> pictureContainsChessboardMap;
	int goodPictures = 0;
	
	for (std::map<int, cv::Mat*>::iterator it = pictureCache.begin(); it != pictureCache.end(); it++)
	{
		if (it->second != 0)
		{
			std::vector<cv::Point2f> corners;
			bool foundAllCorners = cv::findChessboardCorners(*(it->second), boardSize, corners, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);
			
			if (!foundAllCorners)
			{
				ROS_INFO("Took bad picture (id %d)", it->first);
				pictureContainsChessboardMap[it->first] = true;
			}
			else
			{
				ROS_INFO("Took good picture (id %d)", it->first);
				pictureContainsChessboardMap[it->first] = false;
				goodPictures++;
			}
			
			pictureMap[it->first] = it->second;
			
			// Remove image from image cache.
			it->second = 0;
		}
	}
	
	ROS_DEBUG("Got %d good pictures.", goodPictures);
	
	pictureCacheMutex.unlock();
	
	if (goodPictures >= 1) {
		// Create directory for images.
		int error = mkdir("/tmp/calibrationImages", 0777);
		
		if (error != 0 && errno != EEXIST)
		{
			ROS_ERROR("Could not create directory for calibration images (/tmp/calibrationImages): %d", errno);
			
			// Delete images.
			for (std::map<int, cv::Mat*>::iterator it = pictureMap.begin(); it != pictureMap.end(); it++) {
				delete it->second;
			}
			
			return false;
		}
		
		error = mkdir("/tmp/calibrationResult", 0777);
		
		if (error != 0 && errno != EEXIST)
		{
			ROS_ERROR("Could not create directory for calibration images (/tmp/calibrationResult): %d", errno);
			
			// Delete images.
			for (std::map<int, cv::Mat*>::iterator it = pictureMap.begin(); it != pictureMap.end(); it++) {
				delete it->second;
			}
			
			return false;
		}
		
		for (std::map<int, cv::Mat*>::iterator it = pictureMap.begin(); it != pictureMap.end(); it++) {
			std::stringstream ss;
			ss << "/tmp/calibrationImages/cam" << idDict.getForward(it->first) << "_image" << calibrationPictureCount << ".png";
			
			// Save picture on disk for amcctoolbox.
			std::cout << "Saving picture: " << cv::imwrite(ss.str(), *(it->second)) << std::endl;
			
			sensor_msgs::Image img;
			img.width = 640;
			img.height = 480;
			img.step = 3 * 640;
			img.data.reserve(img.step * img.height);
			
			for (int i = 0; i < 640 * 480 * 3; i++)
			{
				img.data[i] = it->second->data[i];
			}
			
			res.images.push_back(img);
			res.containsChessboard.push_back(pictureContainsChessboardMap[it->first]);
			res.ids.push_back(it->first);
			
			delete it->second;
		}
		
		calibrationPictureCount++;
	}
	
	return true;
}

// Service
bool PositionModule::calculateCalibrationCallback(control_application::CalculateCalibration::Request &req, control_application::CalculateCalibration::Response &res)
{
	/*if (!isCalibrating) {
		ROS_ERROR("Cannot calculate calibration! Start calibration first!");
		return false;
	}*/

	if (!idDict.isTranslated()) {
		ROS_WARN("Dictionary was not translated! Translating now.");
		idDict.translateIds();
	}
	
	if (idDict.size() < 2) {
		ROS_ERROR("Have not enough cameras for calibration (Have %d)!", idDict.size());
		isCalibrating = false;
		return false;
	}
	
	if (idDict.size() < 3) {
		ROS_WARN("Have not enough cameras for a useful setup! Have %d, but should be at least 3.", idDict.size());
	}
	
	ROS_INFO("Calculating multi camera calibration. This could take up to 2 hours");
	//ChessboardData data(boardSize.width, boardSize.height, realSize.width, realSize.height);
	ChessboardData data(7, 7, 57, 57);
	
	int camNumber = idDict.size();
	bool ok = trackingWorker.calibrate(&data, camNumber);
	
	if (ok) {
		ROS_INFO("Finished multi camera calibration");
		isCalibrated = true;
	} else {
		ROS_ERROR("Calibration failed!");
	}
	
	for (int i = 0; i < camNumber; i++) {
		Vector position = trackingWorker.getCameraPosition(i);
		res.cameraXPositions.push_back(position.getV1());
		res.cameraYPositions.push_back(position.getV2());
		res.cameraZPositions.push_back(position.getV3());
		
		Matrix rotationMatrix = trackingWorker.getRotationMatrix(i);
		
		res.cameraRotationMatrices.push_back(rotationMatrix.getM11());
		res.cameraRotationMatrices.push_back(rotationMatrix.getM12());
		res.cameraRotationMatrices.push_back(rotationMatrix.getM13());
		res.cameraRotationMatrices.push_back(rotationMatrix.getM21());
		res.cameraRotationMatrices.push_back(rotationMatrix.getM22());
		res.cameraRotationMatrices.push_back(rotationMatrix.getM23());
		res.cameraRotationMatrices.push_back(rotationMatrix.getM31());
		res.cameraRotationMatrices.push_back(rotationMatrix.getM32());
		res.cameraRotationMatrices.push_back(rotationMatrix.getM33());
		
		res.IDs.push_back(idDict.getBackward(i));
		
		// Send calibration data to cameras
		intrinsicsMatrices[idDict.getBackward(i)] = trackingWorker.getIntrinsicsMatrix(i);
		distortionCoefficients[idDict.getBackward(i)] = trackingWorker.getDistortionCoefficients(i);
		
		camera_application::CameraCalibrationData msg;
		msg.ID = idDict.getBackward(i);
		msg.createdByCamera = false;
		
		for (int j = 0; j < 9; j++) {
			msg.intrinsics[j] = intrinsicsMatrices[idDict.getBackward(i)].at<double>(j);
		}
		
		for (int j = 0; j < 4; j++) {
			msg.distortion[j] = distortionCoefficients[idDict.getBackward(i)].at<double>(j);
		}
		
		cameraCalibrationDataPublisher.publish(msg);
		
		// DEBUG: Show calibration results visually
		std::stringstream ss;
		ss << "Calib Results CamId " << idDict.getBackward(i);
		windowNames[idDict.getBackward(i)] = ss.str();
		imageDisplayed[idDict.getBackward(i)] = false;
		
		cv::startWindowThread();
		cv::namedWindow(ss.str());
	}
	
	trackingWorker.updateTrackingArea();
	
	isCalibrating = false;
	
	return ok;
}

// Topic
void PositionModule::pictureCallback(const camera_application::Picture &msg)
{
	// Insert camera id, if not already there.
	idDict.insert(msg.ID);
	
	// DEBUG: Show calibration results visually
	if (imageDisplayed.count(msg.ID) && !imageDisplayed[msg.ID] && intrinsicsMatrices.count(msg.ID) > 0
		&& distortionCoefficients.count(msg.ID) > 0 && windowNames.count(msg.ID) > 0) {
		cv::Mat image(cv::Size(640, 480), CV_8UC3);
		
		for (int i = 0; i < 640 * 480 * 3; i++)	{
			image.data[i] = msg.image[i];
		}
		
		cv::Mat undistorted(cv::Size(640, 480), CV_8UC3);
		cv::undistort(image, undistorted, intrinsicsMatrices[msg.ID], distortionCoefficients[msg.ID]);
		cv::imshow(windowNames[msg.ID], undistorted);
		imageDisplayed[msg.ID] = true;
	}
	
	pictureCacheMutex.lock();
	
	if (isCalibrating) {
		if (pictureCache[msg.ID] != 0) {
			delete pictureCache[msg.ID];
			pictureCache[msg.ID] = 0;
		}
		
		cv::Mat* image = new cv::Mat(cv::Size(640, 480), CV_8UC3);
		
		for (int i = 0; i < 640 * 480 * 3; i++)	{
			image->data[i] = msg.image[i];
		}
		
		pictureCache[msg.ID] = image;
		pictureTimes[msg.ID] = msg.timestamp;
	}
	
	pictureCacheMutex.unlock();
}

// Topic
void PositionModule::systemCallback(const api_application::System &msg)
{
	isRunning = msg.command == 1;
	
	if (isRunning) {
		ROS_INFO("Tracking started");
	} else {
		ROS_INFO("Tracking stopped");
	}
	
	if (!isRunning) {
		int counter = 0;
		
		for (std::vector<long int>::iterator it = timeLog.begin(); it != timeLog.end(); it++) {
			log << counter++ << ", " << *it << std::endl;
		}
		
		cv::destroyAllWindows();
		ros::shutdown();
	}
}

// Topic
void PositionModule::rawPositionCallback(const camera_application::RawPosition &msg)
{
	if (!isCalibrated) {
		return;
	}
	
 	// TODO: Calculate position in our coordinate system.
	// TODO: Is this coordinate change correct for amcctoolbox?
	Vector cameraVector(msg.xPosition, msg.yPosition, 1);
	// ROS_DEBUG("Received position: msg.ID: %d idDict.getForward(msg.ID): %d msg.quadcopterId: %d", msg.ID, idDict.getForward(msg.ID), msg.quadcopterId);
	
	trackingWorker.updatePosition(cameraVector, idDict.getForward(msg.ID), msg.quadcopterId, msg.timestamp);
	
	/*#ifdef QC_PROFILE
	long int trackingClock = getNanoTime();
	#endif
	Vector result = tracker.updatePosition(cameraVector, netIdToCamNo[msg.ID], msg.quadcopterId);
	#ifdef QC_PROFILE
	timeLog.push_back(getNanoTime() - trackingClock);
	#endif
	
	std::vector<Vector> positions;
	std::vector<int> ids;
	std::vector<int> updates;
	positions.push_back(result);
	ids.push_back(msg.quadcopterId);
	updates.push_back(1);
	
	if (result.isValid()) {
		receiver->updatePositions(positions, ids, updates);
	} else {
		ROS_DEBUG("Not enough information to get position of quadcopter %d", msg.quadcopterId);
	}*/
}

void PositionModule::setPictureSendingActivated(bool activated)
{
	camera_application::PictureSendingActivation msg;
	msg.ID = 0;
	msg.active = activated;
	
	pictureSendingActivationPublisher.publish(msg);
}

void PositionModule::sendPing()
{
	api_application::Ping msg;
	msg.ID = rosId;
	pingPublisher.publish(msg);
}

bool PositionModule::isInitialized()
{
	return _isInitialized;
}

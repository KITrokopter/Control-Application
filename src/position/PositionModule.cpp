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

#include "sensor_msgs/Image.h"
#include "api_application/Ping.h"
#include "api_application/Announce.h"

#include "../matlab/Vector.h"

// Use this to test if images are saved properly, when you only have one camera.
//#define SINGLE_CAMERA_CALIBRATION

PositionModule::PositionModule(IPositionReceiver* receiver) : 
	pictureCache(50), // Assume we never have 50 or more modules running on the network.
	pictureTimes(50)
{
	assert(receiver != 0);
	this->receiver = receiver;
	
	ROS_DEBUG("Initializing PositionModule");
	
	_isInitialized = true;
	isCalibrating = false;
	isRunning = false;
	
	ros::NodeHandle n;
	
	this->pictureSendingActivationPublisher = n.advertise<camera_application::PictureSendingActivation>("PictureSendingActivation", 4);
	this->pingPublisher = n.advertise<api_application::Ping>("Ping", 4);
	
	this->pictureSubscriber = n.subscribe("Picture", 128, &PositionModule::pictureCallback, this);
	this->systemSubscriber = n.subscribe("System", 4, &PositionModule::systemCallback, this);
	this->rawPositionSubscriber = n.subscribe("RawPosition", 1024, &PositionModule::rawPositionCallback, this);
	
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
}

PositionModule::~PositionModule()
{
	msg->~KitrokopterMessages();
	
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
	ROS_DEBUG("Taking calibration picture. Have %ld cameras.", camNoToNetId.size());
	
	pictureCacheMutex.lock();
	
	// Build net id -> cam no map.
	if (camNoToNetId.size() != netIdToCamNo.size()) {
		// If already built, there are already images on the disk with wrong ids, so return an error.
		if (netIdToCamNo.size() != 0) {
			ROS_ERROR("Got new cameras after taking first calibration picture!");
			return false;
		}
		
		std::sort(camNoToNetId.begin(), camNoToNetId.end());
		netIdToCamNo.clear();
		
		for (int i = 0; i < camNoToNetId.size(); i++) {
			netIdToCamNo[camNoToNetId[i]] = i;
		}
		
		ROS_INFO("Got %ld cameras", netIdToCamNo.size());
	}
	
	int id = 0;
	std::map<int, cv::Mat*> pictureMap;
	int goodPictures = 0;
	
	for (std::vector<cv::Mat*>::iterator it = pictureCache.begin(); it != pictureCache.end(); it++, id++)
	{
		if (*it != 0)
		{
			std::vector<cv::Point2f> corners;
			bool foundAllCorners = cv::findChessboardCorners(**it, boardSize, corners, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);
			
			if (!foundAllCorners)
			{
				ROS_INFO("Took bad picture (id %d)", id);
			}
			else
			{
				ROS_INFO("Took good picture (id %d)", id);
				goodPictures++;
			}
			
			pictureMap[id] = *it;
			
			// Remove image from image cache.
			*it = 0;
		}
	}
	
	pictureCacheMutex.unlock();
	
	#ifdef SINGLE_CAMERA_CALIBRATION
	if (goodPictures >= 1) {
	#else
	if (goodPictures >= 2) {
	#endif
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
			ss << "/tmp/calibrationImages/cam" << netIdToCamNo[it->first] << "_image" << calibrationPictureCount << ".png";
			
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
	}
	
	if (netIdToCamNo.size() < 2) {
		ROS_ERROR("Have not enough images for calibration (Have %ld)!", netIdToCamNo.size());
	}*/
	
	ROS_INFO("Calculating multi camera calibration. This could take up to 2 hours");
	// ChessboardData data(boardSize.width, boardSize.height, realSize.width, realSize.height);
	ChessboardData data(7, 7, 57, 57);
	
	pictureCacheMutex.lock();
	int camNumber = 3; //camNoToNetId.size();
	pictureCacheMutex.unlock();
	
	// Delete old calibration results.
	system("rm -rf /tmp/calibrationResult/*");
	bool ok = tracker.calibrate(&data, camNumber);
	
	if (ok) {
		ROS_INFO("Finished multi camera calibration");
	} else {
		ROS_ERROR("Calibration failed!");
	}
	
	isCalibrating = false;
	
	return ok;
}

// Topic
void PositionModule::pictureCallback(const camera_application::Picture &msg)
{
	assert(msg.ID < 50);
	
	pictureCacheMutex.lock();
	
	bool idKnown = false;
	
	// Insert camera id, if not already there.
	for (int i = 0; i < camNoToNetId.size(); i++) {
		if (camNoToNetId[i] == msg.ID) {
			idKnown = true;
			break;
		}
	}
	
	if (!idKnown) {
		camNoToNetId.push_back(msg.ID);
	}
	
	if (isCalibrating)
	{
		// Will crash here, if more than 49 modules are used.
		if (pictureCache[msg.ID] != 0)
		{
			delete pictureCache[msg.ID];
			pictureCache[msg.ID] = 0;
			
		}
		
		cv::Mat* image = new cv::Mat(cv::Size(640, 480), CV_8UC3);
		
		for (int i = 0; i < 640 * 480 * 3; i++)
		{
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
		ros::shutdown();
	}
}

// Topic
void PositionModule::rawPositionCallback(const camera_application::RawPosition &msg)
{
 	// TODO: Calculate position in our coordinate system.
	// TODO: Is this coordinate change correct for amcctoolbox?
	Vector cameraVector(msg.xPosition, msg.yPosition, 1);
	ROS_DEBUG("msg.ID: %d netIdToCamNo[msg.ID]: %d msg.quadcopterId: %d", msg.ID, netIdToCamNo[msg.ID], msg.quadcopterId);
	Vector result = tracker.updatePosition(cameraVector, netIdToCamNo[msg.ID], msg.quadcopterId);
	
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
	}
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
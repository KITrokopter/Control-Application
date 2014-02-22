#include "PositionModule.hpp"

#include <assert.h>
#include <sys/stat.h>
#include <errno.h>
#include <stdlib.h>
#include <string>
#include <algorithm>

#include <ros/console.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "sensor_msgs/Image.h"
#include "api_application/Ping.h"
#include "api_application/Announce.h"

#include "../matlab/Vector.h"

PositionModule::PositionModule(IPositionReceiver* receiver)
{
	assert(receiver != 0);
	this->receiver = receiver;
	
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
	ros::ServiceClient announceClient = n.serviceClient<api_application::Announce>("Announce");
	
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
		ROS_DEBUG("PositionModule initialized.");
	}
}

PositionModule::~PositionModule()
{
	msg->~KitrokopterMessages();
	
	// TODO: Free picture cache.
	
	ROS_DEBUG("PositionModule destroyed.");
}

// Service
bool PositionModule::startCalibrationCallback(control_application::StartCalibration::Request &req, control_application::StartCalibration::Response &res)
{
	res.ok = !isCalibrating;
	
	if (!isCalibrating)
	{
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
	}
	
	int id = 0;
	std::map<int, cv::Mat*> goodPictures;
	
	for (std::vector<cv::Mat*>::iterator it = pictureCache.begin(); it != pictureCache.end(); it++, id++)
	{
		if (*it != 0)
		{
			std::vector<cv::Point2f> corners;
			bool foundAllCorners = cv::findChessboardCorners(**it, boardSize, corners, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);
			
			if (!foundAllCorners)
			{
				ROS_INFO("Took bad picture (id %d)", id);
				continue;
			}
			else
			{
				ROS_INFO("Took good picture (id %d)", id);
				goodPictures[id] = *it;
			}
			
			// Remove image from image cache.
			*it = 0;
		}
	}
	
	pictureCacheMutex.unlock();
	
	if (goodPictures.size() >= 2) {
		// Create directory for images.
		int error = mkdir("~/calibrationImages", 770);
		
		if (error != 0 && error != EEXIST)
		{
			ROS_ERROR("Could not create directory for calibration images: %d", error);
			
			// Delete images.
			for (std::map<int, cv::Mat*>::iterator it = goodPictures.begin(); it != goodPictures.end(); it++) {
				delete it->second;
			}
			
			return false;
		}
		
		id = 0;
		for (std::map<int, cv::Mat*>::iterator it = goodPictures.begin(); it != goodPictures.end(); it++) {
			std::stringstream ss;
			ss << "~/calibrationImages/cam" << id << "_image" << calibrationPictureCount << ".png";
			
			cv::imwrite(ss.str(), *(it->second));
			
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
		}
	
		calibrationPictureCount++;
	}
	
	return true;
}

// Service
bool PositionModule::calculateCalibrationCallback(control_application::CalculateCalibration::Request &req, control_application::CalculateCalibration::Response &res)
{
	if (!isCalibrating) {
		ROS_ERROR("Cannot calculate calibration! Start calibration first.");
		return false;
	}
	
	if (netIdToCamNo.size() < 2) {
		ROS_ERROR("Have not enough images for calibration (Have %ld)!", netIdToCamNo.size());
	}
	
	ChessboardData data(boardSize.width, boardSize.height, realSize.width / (boardSize.width - 1), realSize.height / (boardSize.height - 1));
	tracker.calibrate(&data, netIdToCamNo.size());
	
	isCalibrating = false;
	
	system("rm -rf ~/calibrationImages/*");
	
	return true;
}

// Topic
void PositionModule::pictureCallback(const camera_application::Picture &msg)
{
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
		pictureCache.reserve(msg.ID + 1);
		pictureTimes.reserve(msg.ID + 1);
		
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
	
	if (!isRunning) {
		ros::shutdown();
	}
}

// Topic
void PositionModule::rawPositionCallback(const camera_application::RawPosition &msg)
{
 	// TODO: Calculate position in our coordinate system.
	// TODO: Is this coordinate change correct for amcctoolbox?
	Vector cameraVector(1, msg.xPosition, msg.yPosition);
	Vector result = tracker.updatePosition(cameraVector, msg.ID, msg.quadcopterId);
	
	std::vector<Vector> positions;
	std::vector<int> ids;
	std::vector<int> updates;
	positions.push_back(result);
	ids.push_back(msg.quadcopterId);
	updates.push_back(1);
	
	if (result.isValid()) {
		receiver->updatePosition(positions, ids, updates);
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
#include "PositionModule.hpp"

#include <assert.h>

#include <ros/console.h>

#include "sensor_msgs/Image.h"
#include "api_application/Ping.h"
#include "api_application/Announce.h"

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
	
	this->startCalibrationService = n.advertiseService("StartCalibration", &PositionModule::startCalibrationCallback, this);
	this->takeCalibrationPictureService = n.advertiseService("TakeCalibrationPicture", &PositionModule::takeCalibrationPictureCallback, this);
	this->calculateCalibrationService = n.advertiseService("CalculateCalibration", &PositionModule::calculateCalibrationCallback, this);
	
	// Advertise myself to API
	ros::ServiceClient announceClient = n.serviceClient<api_application::Announce>("Announce");
	
	api_application::Announce announce;
	announce.request.type = 3; // 3 means position module
	announce.request.initializeServiceName = std::string("InitializePositionModule");
	
	if (announceClient.call(announce))
	{
		rosId = announce.response.ID;
		
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
	}
	
	isCalibrating = true;
	return true;
}

// Service
bool PositionModule::takeCalibrationPictureCallback(control_application::TakeCalibrationPicture::Request &req, control_application::TakeCalibrationPicture::Response &res)
{
	for (std::vector<cv::Mat*>::iterator it = pictureCache.begin(); it != pictureCache.end(); it++)
	{
		if (*it != 0)
		{
			// TODO: check if image is "good"
			
			sensor_msgs::Image img;
			img.width = 640;
			img.height = 480;
			img.step = 3 * 640;
			
			for (int i = 0; i < 640 * 480 * 3; i++)
			{
				img.data[i] = (*it)->data[i];
			}
			
			res.images.push_back(img);
			
			delete *it;
			*it = 0;
		}
	}
	
	return true;
}

// Service
bool PositionModule::calculateCalibrationCallback(control_application::CalculateCalibration::Request &req, control_application::CalculateCalibration::Response &res)
{
	// TODO: Do stuff with danis code...
	
	return true;
}

// Topic
void PositionModule::pictureCallback(const camera_application::Picture &msg)
{
	if (isCalibrating)
	{
		// Don't know if it works that way and I really can randomly insert now...
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
}

// Topic
void PositionModule::systemCallback(const api_application::System &msg)
{
	isRunning = msg.command == 1;
}

void PositionModule::setPictureSendingActivated(bool activated)
{
	camera_application::PictureSendingActivation msg;
	msg.ID = rosId;
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
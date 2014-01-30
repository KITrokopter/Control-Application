#include "PositionModule.hpp"

#include <assert.h>

#include <ros/console.h>

#include "sensor_msgs/Image.h"

PositionModule::PositionModule(IPositionReceiver* receiver)
{
	assert(receiver != 0);
	this->receiver = receiver;
	
	isCalibrating = false;
	
	ros::NodeHandle n;
	
	this->pictureSendingActivationPublisher = n.advertise<camera_application::PictureSendingActivation>("PictureSendingActivation", 4);
	this->pictureSubscriber = n.subscribe("Picture", 128, &PositionModule::pictureCallback, this);
	
	this->startCalibrationService = n.advertiseService("StartCalibration", &PositionModule::startCalibrationCallback, this);
	this->takeCalibrationPictureService = n.advertiseService("TakeCalibrationPicture", &PositionModule::takeCalibrationPictureCallback, this);
	
	ROS_DEBUG("PositionModule initialized.");
}

PositionModule::~PositionModule()
{
	ROS_DEBUG("PositionModule destroyed.");
}

// Service
bool PositionModule::startCalibrationCallback(control_application::StartCalibration::Request &req, control_application::StartCalibration::Response &res)
{
	res.ok = !isCalibrating;
	
	if (!isCalibrating)
	{
		// Activate picture sending.
		camera_application::PictureSendingActivation msg;
		msg.ID = rosId;
		msg.active = true;
		
		pictureSendingActivationPublisher.publish(msg);
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
}

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
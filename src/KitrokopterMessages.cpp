#include "KitrokopterMessages.hpp"

#include "api_application/Message.h"

KitrokopterMessages::KitrokopterMessages(int id)
{
	this->id = id;
	
	ros::NodeHandle n;
	this->messagePublisher = n.advertise<api_application::Message>("Message", 16);
}

void KitrokopterMessages::message(std::string text)
{
	sendMessage(text, KM_MESSAGE);
}

void KitrokopterMessages::warning(std::string text)
{
	sendMessage(text, KM_WARNING);
}

void KitrokopterMessages::error(std::string text)
{
	sendMessage(text, KM_ERROR);
}

void KitrokopterMessages::sendMessage(std::string text, int type)
{
	api_application::Message msg;
	msg.senderID = id;
	msg.type = type;
	msg.message = text;
	messagePublisher.publish(msg);
}
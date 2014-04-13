#include "KitrokopterMessages.hpp"

#include "api_application/Message.h"

/**
 * Initializes the object to send messages from the specified sender id.
 *
 * @param id The sender id.
 */
KitrokopterMessages::KitrokopterMessages(int id)
{
	this->id = id;

	ros::NodeHandle n;
	this->messagePublisher = n.advertise<api_application::Message>("Message", 16);
}

/**
 * Sends an information message.
 *
 * @param text The message.
 */
void KitrokopterMessages::message(std::string text)
{
	sendMessage(text, KM_MESSAGE);
}

/**
 * Sends a warning message.
 *
 * @param text The message.
 */
void KitrokopterMessages::warning(std::string text)
{
	sendMessage(text, KM_WARNING);
}

/**
 * Sends an error message.
 *
 * @param text The message.
 */
void KitrokopterMessages::error(std::string text)
{
	sendMessage(text, KM_ERROR);
}

/**
 * Sends a message of the given type.
 *
 * @param text The message.
 * @param type The message type. Info = 1, Warning = 2, Error = 3.
 */
void KitrokopterMessages::sendMessage(std::string text, int type)
{
	api_application::Message msg;
	msg.senderID = id;
	msg.type = type;
	msg.message = text;
	messagePublisher.publish(msg);
}
#ifndef KITROKOPTER_MESSAGES_HPP
#define KITROKOPTER_MESSAGES_HPP

#include <string>
#include <ros/ros.h>

// constants
#define KM_MESSAGE 1
#define KM_WARNING 2
#define KM_ERROR 3

/**
 * Class to use the message topic.
 * This allows to send status messages to the user.
 * 
 * @author Sebastian Schmidt
 */
class KitrokopterMessages
{
private:
	int id;
	ros::Publisher messagePublisher;
	
public:
	KitrokopterMessages(int id);
	
	void message(std::string text);
	void warning(std::string text);
	void error(std::string text);
	void sendMessage(std::string text, int type);
};

#endif // KITROKOPTER_MESSAGES_HPP
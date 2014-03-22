#include "AbstractCameraQueue.hpp"

#include <ros/console.h>

void AbstractCameraQueue::enqueue(std::vector<CameraData> data)
{
	for (std::vector<CameraData>::iterator it = data.begin(); it != data.end(); it++) {
		enqueue(*it);
	}
}

void AbstractCameraQueue::enqueue(CameraData data)
{
	enqueueInternal(data);
	
	if (getSize() > 15) {
		ROS_WARN("Camera queue running full. Removing 10 entries");
		
		while (getSize() > 5) {
			dequeue();
		}
	}
}

CameraData AbstractCameraQueue::getInvalidCameraData()
{
	CameraData data;
	data.valid = false;
	return data;
}
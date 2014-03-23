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
		ROS_WARN("Camera queue running full. Quadcopter id: %d", data.quadcopterId);
	}
}

CameraData AbstractCameraQueue::getInvalidCameraData()
{
	CameraData data;
	data.valid = false;
	return data;
}

std::vector<CameraData> AbstractCameraQueue::getInvalidCameraDataVector()
{
	return toVector(getInvalidCameraData());
}

std::vector<CameraData> AbstractCameraQueue::toVector(CameraData data)
{
	std::vector<CameraData> result;
	result.push_back(data);
	return result;
}
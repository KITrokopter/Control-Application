#include "AbstractCameraQueue.hpp"

#include <ros/console.h>

/**
 * Enqueues the content of the given vector of CameraData.
 *
 * @param data A vector of CameraData.
 */
void AbstractCameraQueue::enqueue(std::vector<CameraData> data)
{
	for (std::vector<CameraData>::iterator it = data.begin(); it != data.end(); it++) {
		enqueue(*it);
	}
}

/**
 * Enqueues the given CameraData.
 *
 * @param data The CameraData.
 */
void AbstractCameraQueue::enqueue(CameraData data)
{
	enqueueInternal(data);

	if (getSize() > 50) {
		ROS_WARN("Camera queue running full. Quadcopter id: %d. Entries: %ld", data.quadcopterId, getSize());
	}
}

/**
 * Returns a vector containing the given CameraData.
 *
 * @param data The CameraData.
 * @return A vector containing {@code data}.
 */
std::vector<CameraData> AbstractCameraQueue::toVector(CameraData data)
{
	std::vector<CameraData> result;
	result.push_back(data);
	return result;
}
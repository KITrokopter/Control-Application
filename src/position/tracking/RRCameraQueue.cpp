#include "RRCameraQueue.hpp"

#include <ros/console.h>

RRCameraQueue::RRCameraQueue()
{
	rrIndex = 0;
	size = 0;
}

void RRCameraQueue::enqueueInternal(CameraData data)
{
	if (queues.count(data.camNo) == 0) {
		camNos.push_back(data.camNo);
		queues[data.camNo] = std::queue<CameraData>();
	}
	
	queues[data.camNo].push(data);
	size++;
	
	ROS_DEBUG("Enqueue: size = %ld, queues[data.camNo].size() = %ld", size, queues[data.camNo].size());
}

std::vector<CameraData> RRCameraQueue::dequeue()
{
	if (size == 0) {
		ROS_DEBUG("Size is 0");
		return getInvalidCameraDataVector();
	}
	
	int index = rrIndex;
	
	do {
		index = (index + 1) % camNos.size();
		
		if (queues[camNos[index]].size() > 0) {
			break;
		}
	} while (index != rrIndex);
	
	if (index == rrIndex) {
		ROS_DEBUG("Didn't find CameraData");
		return getInvalidCameraDataVector();
	} else {
		ROS_DEBUG("Found CameraData");
		CameraData result = queues[camNos[index]].front();
		queues[camNos[index]].pop();
		size--;
		
		return toVector(result);
	}
}

bool RRCameraQueue::dataAvailable()
{
	ROS_DEBUG("Size is %ld in dataAvailable()", size);
	return size > 0;
}

size_t RRCameraQueue::getSize()
{
	return size;
}
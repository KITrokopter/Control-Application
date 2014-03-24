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
}

std::vector<CameraData> RRCameraQueue::dequeue()
{
	if (size == 0) {
		return std::vector<CameraData>();
	}
	
	int index = rrIndex;
	int loopCount = 0;
	
	do {
		index = (index + 1) % camNos.size();
		loopCount++;
		
		if (queues[camNos[index]].size() > 0) {
			break;
		}
	} while (index != rrIndex);
	
	if (loopCount > camNos.size()) {
		return std::vector<CameraData>();
	} else {
		CameraData result = queues[camNos[index]].front();
		queues[camNos[index]].pop();
		size--;
		
		return toVector(result);
	}
}

bool RRCameraQueue::dataAvailable()
{
	return size > 0;
}

size_t RRCameraQueue::getSize()
{
	return size;
}
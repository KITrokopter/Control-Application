#include "RRCameraQueue.hpp"

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
		return getInvalidCameraDataVector();
	}
	
	int index = rrIndex;
	
	do {
		if (queues[camNos[index]].size() > 0) {
			break;
		}
		
		index++;
	} while (index != rrIndex);
	
	if (index == rrIndex) {
		return getInvalidCameraDataVector();
	} else {
		rrIndex = (index + 1) % camNos.size();
		CameraData result = queues[camNos[index]].front();
		queues[camNos[index]].pop();
		size--;
		
		return toVector(result);
	}
}

size_t RRCameraQueue::getSize()
{
	return size;
}
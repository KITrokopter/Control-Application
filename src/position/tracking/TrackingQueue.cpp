#include "TrackingQueue.hpp"

TrackingQueue::TrackingQueue()
{
	size = 0;
}

void TrackingQueue::enqueue(std::vector<CameraData> data)
{
	for (std::vector<CameraData>::iterator it = data.begin(); it != data.end(); it++) {
		enqueue(*it);
	}
}

void TrackingQueue::enqueue(CameraData data)
{
	if (queues.count(data.quadcopterId) == 0) {
		queues[data.quadcopterId] = T;
		ids.push_back(data.quadcopterId);
	}
	
	size++;
}

size_t TrackingQueue::getSize()
{
	return size;
}

std::vector<CameraData> dequeue()
{
	if (size == 0) {
		return std::vector<CameraData>();
	}
	
	int index = rrIndex;
	
	do {
		if (queues[ids[index]].size() > 0) {
			break;
		}
		
		index++;
	} while (index != rrIndex);
	
	if (index == rrIndex) {
		return std::vector<CameraData>();
	} else {
		rrIndex = (index + 1) % ids.size();
		vector<CameraData> result = queues[ids[index]].front();
		queues[ids[index]].pop();
		size -= result.size();
		
		return result;
	}
}
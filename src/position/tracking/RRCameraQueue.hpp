#pragma once

#include "AbstractCameraQueue.hpp"

#include <queue>
#include <map>

class RRCameraQueue : public AbstractCameraQueue {
public:
	RRCameraQueue();
	
	// Implementations of AbstractCameraQueue
	size_t getSize();
	std::vector<CameraData> dequeue();
	
protected:
	// Implementations of AbstractCameraQueue
	void enqueueInternal(CameraData data);
	
private:
	size_t size;
	size_t rrIndex;
	std::map<int, std::queue<CameraData> > queues;
	std::vector<int> camNos;
};
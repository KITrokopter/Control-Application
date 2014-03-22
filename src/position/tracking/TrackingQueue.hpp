#pragma once

#include "AbstractCameraQueue.hpp"

template <class T>
class TrackingQueue {
public:
	TrackingQueue();
	
	void enqueue(std::vector<CameraData> data);
	void enqueue(CameraData data);
	
	size_t getSize();
	std::vector<CameraData> dequeue();
	
private:
	std::map<int, AbstractCameraQueue> queues;
	std::vector<int> ids;
	int rrIndex;
	size_t size;
};
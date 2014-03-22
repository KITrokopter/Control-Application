#pragma once

#include "AbstractCameraQueue.hpp"

#include <map>

template <class T>
class TrackingQueue {
public:
	// Template stuff
	TrackingQueue()
	{
		size = 0;
	}
	
	~TrackingQueue()
	{
		for (std::map<int, AbstractCameraQueue*>::iterator it = queues.begin(); it != queues.end(); it++) {
			delete it->second;
		}
	}
	
	void enqueue(std::vector<CameraData> data);
	void enqueue(CameraData data);
	
	size_t getSize();
	std::vector<CameraData> dequeue();
	bool dataAvailable();
	
private:
	std::map<int, AbstractCameraQueue*> queues;
	std::vector<int> ids;
	int rrIndex;
	size_t size;
};
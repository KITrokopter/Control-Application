#pragma once

#include "AbstractCameraQueue.hpp"

#include <list>

class SynchronousCameraQueue : public AbstractCameraQueue {
public:
	SynchronousCameraQueue(long int arrivalDelay, long int maxDelay);
	
	// Implementations of AbstractCameraQueue
	size_t getSize();
	bool dataAvailable();
	std::vector<CameraData> dequeue();
	
protected:
	// Implementations of AbstractCameraQueue
	void enqueueInternal(CameraData data);
	
private:
	class Bucket {
	public:
		CameraData data;
		long int arrivalTime;
		
		bool operator < (Bucket b) {
			return data.time < b.data.time;
		}
	};
	
	int maxDelay;
	int arrivalDelay;
	
	size_t size;
	std::list<Bucket> queue;
};
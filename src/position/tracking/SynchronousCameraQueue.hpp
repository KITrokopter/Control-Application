#pragma once

#include "AbstractCameraQueue.hpp"

#include <list>
#include <set>

class SynchronousCameraQueue : public AbstractCameraQueue {
public:
	SynchronousCameraQueue(long int arrivalDelay, long int maxDelay, long int maxGroupInterval);
	
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
		
		bool operator < (Bucket b)
		{
			return data.time < b.data.time;
		}
	};
	
	class Group {
	public:
		Group()
		{
			valid = false;
			value = 0;
		}
		
		bool valid;
		long int value;
		std::vector<CameraData> data;
	};
	
	Group searchGroup(std::list<Bucket>::iterator it, long int currentTime, std::list<Bucket>::iterator begin, std::list<Bucket>::iterator end);
	void calculateValue(Group *group, long int minTime, long int maxTime, long int currentTime, bool hasWaiting);
	
	long int maxDelay;
	long int arrivalDelay;
	long int maxGroupInterval;
	
	size_t size;
	std::list<Bucket> queue;
	std::set<int> camNos;
};
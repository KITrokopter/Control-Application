#include "SynchronousCameraQueue.hpp"

#include "../../matlab/profiling.hpp"

SynchronousCameraQueue::SynchronousCameraQueue(long int arrivalDelay, long int maxDelay)
{
	this->maxDelay = maxDelay;
	this->arrivalDelay = arrivalDelay;
	size = 0;
}

size_t SynchronousCameraQueue::getSize()
{
	return size;
}

bool SynchronousCameraQueue::dataAvailable()
{
	long int currentTime = getNanoTime();
	
	if (queue.size() == 0) {
		return false;
	}
	
	for (std::list<Bucket>::iterator it = queue.end(); it != queue.begin(); it--) {
		if (currentTime - it->arrivalTime > maxDelay) {
			return true;
		}
		
		if (currentTime - it->arrivalTime > arrivalDelay && queue.size() > 2) {
			return true;
		}
	}
	
	return false;
}

void SynchronousCameraQueue::enqueueInternal(CameraData data)
{
	Bucket b;
	b.data = data;
	b.arrivalTime = getNanoTime();
	
	std::list<Bucket> tmp;
	tmp.push_back(b);
	queue.merge(tmp);
	
	size++;
}
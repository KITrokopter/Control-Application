#include "SynchronousCameraQueue.hpp"

#include "../../matlab/profiling.hpp"

SynchronousCameraQueue::SynchronousCameraQueue(long int arrivalDelay, long int maxDelay, long maxGroupInterval)
{
	this->maxDelay = maxDelay;
	this->arrivalDelay = arrivalDelay;
	this->maxGroupInterval = maxGroupInterval;
	
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
	
	camNos.insert(data.camNo);
	
	size++;
}

std::vector<CameraData> SynchronousCameraQueue::dequeue()
{
	long int currentTime = getNanoTime();
	
	// Search for element which is older than maxDelay
	for (std::list<Bucket>::iterator it = queue.end(); it != queue.begin(); it--) {
		if (currentTime - it->arrivalTime > maxDelay) {
			// element is overdue, so return group, even if it's invalid.
			return searchGroup(it, currentTime, queue.begin(), queue.end()).data;
		}
	}
	
	Group result;
	
	for (std::list<Bucket>::iterator it = queue.begin(); it != queue.end(); it++) {
		Group tmp = searchGroup(it, currentTime, queue.begin(), queue.end());
		
		if (tmp.valid && (!result.valid || tmp.value > result.value)) {
			result = tmp;
		}
	}
	
	if (result.valid) {
		return result.data;
	} else {
		return std::vector<CameraData>();
	}
}

SynchronousCameraQueue::Group SynchronousCameraQueue::searchGroup(std::list<Bucket>::iterator it, long int currentTime, std::list<Bucket>::iterator begin, std::list<Bucket>::iterator end)
{
	Group result;
	result.data.push_back(it->data);
	
	std::set<int> usedCamNos;
	camNos.insert(it->data.camNo);
	
	// true if we found an element that is older than arrivalDelay
	bool hasWaiting = currentTime - it->arrivalTime > arrivalDelay;
	
	// The borders of the time interval of the group we are constructing
	long int minTime = it->data.time;
	long int maxTime = it->data.time;
	
	// The current rand of the search area
	std::list<Bucket>::iterator left = it;
	std::list<Bucket>::iterator right = it;
	
	// Temp iterator to reduce the amount of if/else
	std::list<Bucket>::iterator current = it;
	
	// Which side of the rand to go
	int direction = 0; // -1 left, 1 right, 0 terminate
	
	do {
		direction = 0;
		
		// Choose direction
		if (left != begin && right != end) {
			// Both directions are possible
			
			left--;
			right++;
			
			// Prioritize the values that'd result in a smaller range
			if (minTime - left->data.time > right->data.time - maxTime) {
				left++;
				direction = 1;
				current = right;
			} else {
				right--;
				direction = -1;
				current = left;
			}
			
		// Not both directions are possible
		// Go to the direction that is possible
		} else if (left != begin) {
			direction = -1;
			current = left;
		} else if (right != end) {
			direction = 1;
			current = right;
		}
		
		// Check if the interval size would still be below maxGroupInterval if the next element would be added
		// Abort if the interval size cannot be enlarged
		if (direction == 1 && current->data.time - minTime > maxGroupInterval
			|| direction == -1 && maxTime - current->data.time > maxGroupInterval)
		{
			direction = 0;
		}
		
		if (direction != 0 && usedCamNos.count(current->data.camNo) == 0) {
			// Check if we found an element that has completed the arrivalDelay
			if (currentTime - current->arrivalTime > arrivalDelay) {
				hasWaiting = true;
			}
			
			if (direction == 1) {
				// Going right, increase right interval border
				maxTime = current->data.time;
			} else {
				// Going left, decrease left interval border
				minTime = current->data.time;
			}
			
			// Add to results, mark camNo as used and put CameraData into result vector
			usedCamNos.insert(current->data.camNo);
			result.data.push_back(current->data);
			
			// Result has now >= 2 elements, so it is valid
			result.valid = true;
			
			// If no more other cameras available, abort
			if (result.data.size() == camNos.size()) {
				direction = 0;
			}
		}
	} while (direction != 0);
	
	calculateValue(&result, minTime, maxTime, currentTime, hasWaiting);
	
	return result;
}

void SynchronousCameraQueue::calculateValue(Group *group, long int minTime, long int maxTime, long int currentTime, bool hasWaiting)
{
	long int value = 2 * maxGroupInterval - (maxTime - minTime);
	
	if (hasWaiting || group->data.size() == camNos.size()) {
		value *= group->data.size();
	}
	
	double timeValue = minTime - (currentTime - arrivalDelay);
	timeValue /= maxDelay - arrivalDelay;
	timeValue *= maxGroupInterval;
	
	value += (long int) timeValue;
	
	group->value = value;
}
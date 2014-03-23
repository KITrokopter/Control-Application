#include "SynchronousCameraQueue.hpp"

#include <ros/console.h>

#include "../../matlab/profiling.hpp"

SynchronousCameraQueue::SynchronousCameraQueue(long int arrivalDelay, long int maxDelay, long maxGroupInterval)
{
	this->maxDelay = maxDelay;
	this->arrivalDelay = arrivalDelay;
	this->maxGroupInterval = maxGroupInterval;
	lastArrivalTime = 0;
	minimumPictureTime = 0;
}

size_t SynchronousCameraQueue::getSize()
{
	return queue.size();
}

bool SynchronousCameraQueue::dataAvailable()
{
	long int currentTime = getNanoTime();
	
	if (queue.size() == 0) {
		return false;
	}
	
	if (queue.size() >= camNos.size()) {
		return true;
	}
	
	for (std::list<Bucket>::reverse_iterator it = queue.rbegin(); it != queue.rend(); it++) {
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
	// Make it impossible to insert data for a time earlier than the last dequeue() result
	if (data.time <= minimumPictureTime) {
		return;
	}
	
	Bucket b;
	b.data = data;
	b.arrivalTime = getNanoTime();
	
	if (b.arrivalTime <= lastArrivalTime) {
		lastArrivalTime++;
		b.arrivalTime = lastArrivalTime;
	}
	
	std::list<Bucket> tmp;
	tmp.push_back(b);
	queue.merge(tmp);
	
	camNos.insert(data.camNo);
}

std::vector<CameraData> SynchronousCameraQueue::dequeue()
{
	static bool once = true;
	
	if (once) {
		ROS_DEBUG("SynchronousCameraQueue::dequeue(): enter first");
		once = false;
	} else {
		ROS_DEBUG("SynchronousCameraQueue::dequeue(): enter");
	}
	
	long int currentTime = getNanoTime();
	
	Group result;
	
	ROS_DEBUG("SynchronousCameraQueue::dequeue(): Searching best valid result");
	for (std::list<Bucket>::iterator it = queue.begin(); it != queue.end(); it++) {
		Group tmp = searchGroup(it, currentTime, queue.begin(), --queue.end());
		
		if (tmp.isValid() && (!result.isValid() || tmp.getValue() > result.getValue())) {
			ROS_DEBUG("SynchronousCameraQueue::dequeue(): Found better result");
			result = tmp;
		} else {
			ROS_DEBUG("SynchronousCameraQueue::dequeue(): Didn't find better result");
		}
	}
	
	ROS_DEBUG("SynchronousCameraQueue::dequeue(): Check if result was found");
	if (result.isValid()) {
		cutOffQueue(result.getYoungest());
		
		ROS_DEBUG("SynchronousCameraQueue::dequeue(): Found valid result");
		return result.getData();
	} else {
		// Search for element which is older than maxDelay
		for (std::list<Bucket>::iterator it = queue.begin(); it != queue.end(); it++) {
			if (currentTime - it->arrivalTime > maxDelay) {
				ROS_WARN("Strange things are happening! The only thing to return is old and lonely.");
				
				// element is overdue and nothing else was found, so return group, even if it's invalid.
				result = searchGroup(it, currentTime, queue.begin(), --queue.end());
				cutOffQueue(result.getYoungest());
				
				ROS_DEBUG("SynchronousCameraQueue::dequeue(): Found invalid result");
				return result.getData();
			}
		}
		
		ROS_DEBUG("SynchronousCameraQueue::dequeue(): Found no result");
		return std::vector<CameraData>();
	}
}

SynchronousCameraQueue::Group SynchronousCameraQueue::searchGroup(std::list<Bucket>::iterator it, long int currentTime, std::list<Bucket>::iterator begin, std::list<Bucket>::iterator end)
{
	ROS_DEBUG("SynchronousCameraQueue::searchGroup(): enter");
	ROS_DEBUG("SynchronousCameraQueue::searchGroup(): it->data.time = %ld", it->data.time);
	
	Group result(it, currentTime, arrivalDelay, maxDelay, maxGroupInterval, camNos.size());
	
	std::set<int> usedCamNos;
	camNos.insert(it->data.camNo);
	
	// The current rand of the search area
	std::list<Bucket>::iterator left = it;
	std::list<Bucket>::iterator right = it;
	
	// Temp iterator to reduce the amount of if/else
	std::list<Bucket>::iterator current = it;
	
	// Which side of the rand to go
	int direction = 0; // -1 left, 1 right, 0 terminate
	
	do {
		ROS_DEBUG("SynchronousCameraQueue::searchGroup(): Enter loop");
		
		direction = 0;
		
		// Choose direction
		if (left != begin && right != end) {
			// Both directions are possible
			ROS_DEBUG("SynchronousCameraQueue::searchGroup(): No rand reached");
			
			left--;
			right++;
			
			// Prioritize the values that'd result in a smaller range
			if (result.getMinTime() - left->data.time > right->data.time - result.getMaxTime()) {
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
			ROS_DEBUG("SynchronousCameraQueue::searchGroup(): Right rand reached");
			direction = -1;
			current = --left;
		} else if (right != end) {
			ROS_DEBUG("SynchronousCameraQueue::searchGroup(): Left rand reached");
			direction = 1;
			current = ++right;
		}
		
		ROS_DEBUG("SynchronousCameraQueue::searchGroup(): Direction is %d", direction);
		
		// Check if the interval size would still be below maxGroupInterval if the next element would be added
		// Abort if the interval size cannot be enlarged
		if (result.wouldInvalidateGroup(current)) {
			ROS_DEBUG("SynchronousCameraQueue::searchGroup(): Would invalidate group");
			direction = 0;
		}
		
		if (direction != 0 && usedCamNos.count(current->data.camNo) == 0) {
			// Mark camNo as used and add to result
			usedCamNos.insert(current->data.camNo);
			result.add(current);
			
			// If no more other cameras available, abort
			if (result.getData().size() == camNos.size()) {
				direction = 0;
			}
		}
	} while (direction != 0);
	
	result.calculateValue();
	
	ROS_DEBUG("SynchronousCameraQueue::searchGroup(): exit");
	return result;
}

void SynchronousCameraQueue::cutOffQueue(std::list<Bucket>::iterator it)
{
	ROS_DEBUG("SynchronousCameraQueue::cutOffQueue(): it->arrivalTime: %ld", it->arrivalTime);
	ROS_DEBUG("SynchronousCameraQueue::cutOffQueue(): queue.begin()->arrivalTime: %ld", queue.begin()->arrivalTime);
	
	minimumPictureTime = it->data.time;
	
	std::list<Bucket> toDelete;
	toDelete.splice(toDelete.begin(), queue, queue.begin(), it);
	queue.pop_front();
	
	int count = 0;
	for (std::list<Bucket>::iterator i = queue.begin(); i != queue.end(); i++) {
		ROS_DEBUG("SynchronousCameraQueue::cutOffQueue(): %ld", i->arrivalTime);
		count++;
	}
	
	ROS_DEBUG("SynchronousCameraQueue::cutOffQueue(): %d entries in queue after splice", count);
	
	count = 0;
	for (std::list<Bucket>::iterator i = toDelete.begin(); i != toDelete.end(); i++) {
		ROS_DEBUG("SynchronousCameraQueue::cutOffQueue(): %ld", i->arrivalTime);
		count++;
	}
	
	ROS_DEBUG("SynchronousCameraQueue::cutOffQueue(): %d entries in toDelete after splice", count);
}
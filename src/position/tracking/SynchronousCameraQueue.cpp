#include "SynchronousCameraQueue.hpp"

#include <ros/console.h>

#include "../../matlab/profiling.hpp"

SynchronousCameraQueue::SynchronousCameraQueue(long int arrivalDelay, long int maxDelay, long maxGroupInterval, bool ensureCam0)
{
	this->maxDelay = maxDelay;
	this->arrivalDelay = arrivalDelay;
	this->maxGroupInterval = maxGroupInterval;
	this->ensureCam0 = ensureCam0;
	lastArrivalTime = 0;
	minimumPictureTime = 0;
	
	ROS_INFO("SynchronousCameraQueue(): arrivalDelay = %.2fms, maxDelay = %.2fms, maxGroupInterval = %.2fms, ensureCam0 = %s", arrivalDelay / 1e6, maxDelay / 1e6, maxGroupInterval / 1e6, ensureCam0 ? "true" : "false");
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
		ROS_DEBUG("Dropped data from cam %d, copter %d because it was too old (time: %ld)", data.camNo, data.quadcopterId, data.time);
		return;
	} else {
		// ROS_DEBUG("Inserting data from cam %d, copter %d (time: %ld)", data.camNo, data.quadcopterId, data.time);
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
	long int currentTime = getNanoTime();
	
	Group result;
	
	for (std::list<Bucket>::iterator it = queue.begin(); it != queue.end(); it++) {
		Group tmp = searchGroup(it, currentTime, queue.begin(), --queue.end());
		
		if (tmp.isValid() && (!result.isValid() || tmp.getValue() > result.getValue())) {
			result = tmp;
		}
	}
	
	if (result.isValid()) {
		//printQueue();
		cutOffQueue(result.getYoungest());
		
		if (result.getData().size() < camNos.size()) {
			// TODO uncomment
			//ROS_WARN("Quadcopter %d is only seen by %ld cameras, but there are %ld entries in the queue", result.getData()[0].quadcopterId, result.getData().size(), queue.size());
		}
		
		return result.getData();
	} else {
		// Search for element which is older than maxDelay
		for (std::list<Bucket>::iterator it = queue.begin(); it != queue.end(); it++) {
			if (currentTime - it->arrivalTime > maxDelay) {
				// TODO uncomment
				//ROS_WARN("The only thing to return is old and probably invalid, but better than nothing so we return it anyways.");
				
				// element is overdue and nothing else was found, so return group, even if it's invalid.
				result = searchGroup(it, currentTime, queue.begin(), --queue.end());
				//printQueue();
				cutOffQueue(result.getYoungest());
				
				return result.getData();
			}
		}
		
		return std::vector<CameraData>();
	}
}

SynchronousCameraQueue::Group SynchronousCameraQueue::searchGroup(std::list<Bucket>::iterator it, long int currentTime, std::list<Bucket>::iterator begin, std::list<Bucket>::iterator end)
{
	Group result(it, currentTime, arrivalDelay, maxDelay, maxGroupInterval, camNos.size(), ensureCam0);
	
	std::set<int> usedCamNos;
	usedCamNos.insert(it->data.camNo);
	
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
			direction = -1;
			current = --left;
		} else if (right != end) {
			direction = 1;
			current = ++right;
		}
		
		// Check if the interval size would still be below maxGroupInterval if the next element would be added
		// Abort if the interval size cannot be enlarged
		if (result.wouldInvalidateGroup(current)) {
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
	
	return result;
}

void SynchronousCameraQueue::cutOffQueue(std::list<Bucket>::iterator it)
{
	minimumPictureTime = it->data.time;
	
	std::list<Bucket> toDelete;
	toDelete.splice(toDelete.begin(), queue, queue.begin(), it);
	queue.pop_front();
}

void SynchronousCameraQueue::printQueue()
{
	ROS_DEBUG("Printing list");
	
	for (std::list<Bucket>::iterator it = queue.begin(); it != queue.end(); it++) {
		ROS_DEBUG("Cam %d, Copter %d: arrival %ld, snaptime %ld", it->data.camNo, it->data.quadcopterId, it->arrivalTime, it->data.time);
	}
}

#include "SynchronousCameraQueue.hpp"

#include <ros/console.h>

SynchronousCameraQueue::Group::Group()
{
	canBeValid = false;
	currentTime = 0;
	arrivalDelay = 0;
	maxDelay = 0;
	maxGroupInterval = 0;
	value = 0;
	minTime = 0;
	maxTime = 0;
}

SynchronousCameraQueue::Group::Group(std::list<Bucket>::iterator it, long int currentTime, long int arrivalDelay, long int maxDelay, long int maxGroupInterval, long int cameraCount)
{
	this->currentTime = currentTime;
	this->arrivalDelay = arrivalDelay;
	this->maxDelay = maxDelay;
	this->maxGroupInterval = maxGroupInterval;
	this->cameraCount = cameraCount;
	youngest = it;
	oldest = it;
	data.push_back(it->data);
	
	value = 0;
	canBeValid = true;
	minTime = it->data.time;
	maxTime = it->data.time;
}

void SynchronousCameraQueue::Group::calculateValue()
{
	// First, define base value by interval length times camera count
	long int value = 2 * maxGroupInterval - (maxTime - minTime);
	value *= data.size();
	//value += maxGroupInterval;
	
	// Calculate another value defined by the age of the oldest CameraData
	double timeValue = minTime - (currentTime - arrivalDelay);
	timeValue /= maxDelay - arrivalDelay;
	timeValue *= maxGroupInterval;
	
	// Combine base and time value
	value += (long int) timeValue;
	
	// Update value of given group object
	this->value = value;
}

void SynchronousCameraQueue::Group::add(std::list<Bucket>::iterator it)
{
	if (it->data.time < minTime) {
		minTime = it->data.time;
	} else if (it->data.time > maxTime) {
		maxTime = it->data.time;
	}
	
	if (it->arrivalTime > youngest->arrivalTime) {
		youngest = it;
	}
	
	if (it->arrivalTime < oldest->arrivalTime) {
		oldest = it;
	}
	
	data.push_back(it->data);
}

bool SynchronousCameraQueue::Group::isValid()
{
	return canBeValid && data.size() >= 2 && (hasWaiting() || data.size() == cameraCount);
}

long int SynchronousCameraQueue::Group::getMinTime()
{
	return minTime;
}

long int SynchronousCameraQueue::Group::getMaxTime()
{
	return maxTime;
}

bool SynchronousCameraQueue::Group::wouldInvalidateGroup(std::list<Bucket>::iterator it)
{
	return it->data.time - minTime > maxGroupInterval || maxTime - it->data.time > maxGroupInterval;
}

long int SynchronousCameraQueue::Group::getValue()
{
	return value;
}

std::vector<CameraData> SynchronousCameraQueue::Group::getData()
{
	return data;
}

bool SynchronousCameraQueue::Group::hasWaiting()
{
	if (!canBeValid) {
		return false;
	}
	
	return currentTime - oldest->arrivalTime > arrivalDelay;
}

std::list<SynchronousCameraQueue::Bucket>::iterator SynchronousCameraQueue::Group::getYoungest()
{
	return youngest;
}
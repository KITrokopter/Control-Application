#include "SynchronousCameraQueue.hpp"

#include <ros/console.h>

/**
 * Creates a new emtpy Group.
 */
SynchronousCameraQueue::Group::Group()
{
	canBeValid = false;
	ensureCam0 = false;
	currentTime = 0;
	arrivalDelay = 0;
	maxDelay = 0;
	maxGroupInterval = 0;
	value = 0;
	minTime = ~1;
	maxTime = 0;
}

/**
 * Creates a new Group containing the given element.
 *
 * @param it An iterator pointing to the element to add to the group.
 * @param currentTime The time that should be used as 'now' for
 * validity checks.
 * @param arrivalDelay The arrivalDelay of the SynchronousCameraQueue.
 * @param maxDelay The maxDelay of the SynchronousCameraQueue.
 * @param maxGroupInterval The maxGroupInterval of the SynchronousCameraQueue.
 * @param cameraCount The amount of cameras known to the SynchronousCameraQueue.
 * @param ensureCam0 The ensureCam0 parameter of the SynchronousCameraQueue.
 * @see SynchronousCameraQueue
 */
SynchronousCameraQueue::Group::Group(std::list<Bucket>::iterator it, long int currentTime, long int arrivalDelay, long int maxDelay, long int maxGroupInterval, long int cameraCount, bool ensureCam0)
{
	this->currentTime = currentTime;
	this->arrivalDelay = arrivalDelay;
	this->maxDelay = maxDelay;
	this->maxGroupInterval = maxGroupInterval;
	this->cameraCount = cameraCount;
	this->ensureCam0 = ensureCam0;
	youngest = it;
	oldest = it;
	data.push_back(it->data);

	value = 0;
	canBeValid = true;
	minTime = it->data.time;
	maxTime = it->data.time;
}

/**
 * Updates the value of this group.
 *
 * The function tries to ensure, that groups with more
 * cameras and a smaller interval between smallest and
 * largest CameraData creation time are more valuable.
 * Also, the value is decreased, the older a Goup
 * becomes.
 */
void SynchronousCameraQueue::Group::calculateValue()
{
	// First, define base value by interval length times camera count
	long int value = 2 * maxGroupInterval - (maxTime - minTime);
	value *= data.size();
	//value += maxGroupInterval;

	// Calculate another value defined by the age of the oldest CameraData.
	// This makes older groups less valuable.
	double timeValue = minTime - (currentTime - arrivalDelay);
	timeValue /= maxDelay - arrivalDelay;
	timeValue *= maxGroupInterval;

	// Combine base and time value
	value += (long int) timeValue;

	// Update value of this object
	this->value = value;
}

/**
 * Adds an element to the group.
 *
 * @param it An iterator pointing to the element to add.
 */
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

	if (maxTime - minTime > maxGroupInterval) {
		canBeValid = false;
	}

	data.push_back(it->data);
}

/**
 * Returns true if the group is valid.
 *
 * A group is valid if all these conditions are met:
 * <ul>
 * <li>The group was not created with the default constructor.</li>
 * <li>The difference between the youngest CameraData and the
 *     oldest CameraData is smaller or equal to
 *     maxGroupInterval.</li>
 * <li>The group contains at least two entries.</li>
 * <li>The group has at least one CameraData that is longer in the
 *     queue than arrivalDelay, OR it has one CameraData per
 *     camera.</li>
 * <li>The ensureCam0 flag is false, OR it is true and the group contains
 *     CameraData of camera 0.</li>
 * </ul>
 *
 * @return True if the group is valid, false otherwise.
 */
bool SynchronousCameraQueue::Group::isValid()
{
	// Check if cam 0 is in the list
	bool cam0Found = false;
	for (std::vector<CameraData>::iterator it = data.begin(); it != data.end(); it++) {
		if (it->camNo == 0) {
			cam0Found = true;
		}
	}

	return canBeValid && data.size() >= 2 && (hasWaiting() || data.size() == cameraCount) && (cam0Found || !ensureCam0);
}


/**
 * Returns the smallest CameraData creation time in this group.
 *
 * @return The smallest CameraData creation time in this group.
 */
long int SynchronousCameraQueue::Group::getMinTime()
{
	return minTime;
}

/**
 * Returns the largest CameraData creation time in this group.
 *
 * @return The largest CameraData creation time in this group.
 */
long int SynchronousCameraQueue::Group::getMaxTime()
{
	return maxTime;
}

/**
 * Checks if adding the given CameraData would invalidate the group.
 * That is, if the difference between the youngest CameraData and
 * the oldest CameraData would be greater than maxGroupInterval
 * after adding the given CameraData.
 *
 * @param it An iterator pointing to the CameraData to check.
 * @return True, if the given CameraData would invalidate the
 * group by adding it, false otherwise.
 */
bool SynchronousCameraQueue::Group::wouldInvalidateGroup(std::list<Bucket>::iterator it)
{
	return it->data.time - minTime > maxGroupInterval || maxTime - it->data.time > maxGroupInterval;
}

/**
 * Returns the value of this group. The value has to be calculated by
 * {@link #calculateValue()} before using this, and is invalidated by
 * a call to {@link #add(std::list<Bucket>::iterator)}.
 *
 * @return The value of this group.
 */
long int SynchronousCameraQueue::Group::getValue()
{
	return value;
}

/**
 * Returns the CameraData in this group as vector.
 *
 * @return The CameraData in this group.
 */
std::vector<CameraData> SynchronousCameraQueue::Group::getData()
{
	return data;
}

/**
 * Checks if at least one of the CameraData in this group
 * is in the waiting phase.
 *
 * @return True if at least one of the CameraData in this
 * group is older than arrivalDelay.
 */
bool SynchronousCameraQueue::Group::hasWaiting()
{
	if (!canBeValid) {
		return false;
	}

	return currentTime - oldest->arrivalTime > arrivalDelay;
}

/**
 * Returns the yougnest CameraData of the group.
 *
 * @return The youngest CameraData of the group.
 */
std::list<SynchronousCameraQueue::Bucket>::iterator SynchronousCameraQueue::Group::getYoungest()
{
	return youngest;
}
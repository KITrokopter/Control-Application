#pragma once

#include <map>
#include <ros/console.h>

#include "AbstractCameraQueue.hpp"

/**
 * Uses one AbstractCameraQueue per quadcopter to schedule
 * the camera data. Dequeueing is done round-robin style
 * for the quadcopter id. The camera data itself is selected
 * by the underlying AbstractCameraQueue.
 *
 * @author Sebastian Schmidt
 * @param T The implementation of AbstractCameraQueue to use.
 */
template <class T>
class TrackingQueue {
public:
	TrackingQueue();
	~TrackingQueue();

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

/**
 * Creates a new empty TrackingQueue.
 */
template <class T>
TrackingQueue<T>::TrackingQueue()
{
	size = 0;
	rrIndex = 0;
}

/**
 * Deletes the TrackingQueue.
 */
template <class T>
TrackingQueue<T>::~TrackingQueue()
{
	for (std::map<int, AbstractCameraQueue*>::iterator it = queues.begin(); it != queues.end(); it++) {
		delete it->second;
	}
}

/**
 * Enqueues the given CameraData.
 *
 * @param data A vector containing CameraData.
 */
template <class T>
void TrackingQueue<T>::enqueue(std::vector<CameraData> data)
{
	for (std::vector<CameraData>::iterator it = data.begin(); it != data.end(); it++) {
		enqueue(*it);
	}
}

/**
 * Enqueues the given CameraData.
 *
 * @param data The CameraData.
 */
template <class T>
void TrackingQueue<T>::enqueue(CameraData data)
{
	if (queues.count(data.quadcopterId) == 0) {
		// Since the RRCameraQueue is missing this constructor, this part has to be
		// edited if it should be used.
		queues[data.quadcopterId] = new T(15L * 100 * 1000 * 1000 / 30,
										  2L * 1000 * 1000 * 1000 / 30,
										  2L * 1000 * 1000 * 1000 / 30,
										  true);
		ids.push_back(data.quadcopterId);
	}

	size++;
	queues[data.quadcopterId]->enqueue(data);
}

/**
 * Returns the amount of CameraData that is currently enqueued.
 *
 * @return The current size of the queue.
 */
template <class T>
size_t TrackingQueue<T>::getSize()
{
	return size;
}

/**
 * Dequeues one set of CameraData from the queue.
 * The quadcopter ids are run through round-robin style.
 * Per quadcopter, the underlying AbstractCameraQueue is used.
 *
 * @return A set of CameraData to be evaluated.
 */
template <class T>
std::vector<CameraData> TrackingQueue<T>::dequeue()
{
	if (size == 0) {
		return std::vector<CameraData>();
	}

	int index = rrIndex;
	int loopCount = 0;

	do {
		index  = (index + 1) % ids.size();
		loopCount++;

		if (queues[ids[index]]->dataAvailable()) {
			break;
		}
	} while (index != rrIndex);

	if (loopCount > ids.size()) {
		return std::vector<CameraData>();
	} else {
		rrIndex = index;
		std::vector<CameraData> result = queues[ids[index]]->dequeue();
		size -= result.size();

		return result;
	}
}

/**
 * Returns true if one of the underlying AbstractCameraQueues
 * return true for this function.
 *
 * @return True, if one of the underlying camera queues has data available.
 */
template <class T>
bool TrackingQueue<T>::dataAvailable()
{
	for (std::map<int, AbstractCameraQueue*>::iterator it = queues.begin(); it != queues.end(); it++) {
		if (it->second->dataAvailable()) {
			return true;
		}
	}

	return false;
}

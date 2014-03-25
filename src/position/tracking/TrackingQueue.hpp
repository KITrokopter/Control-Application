#pragma once

#include <map>
#include <ros/console.h>

#include "AbstractCameraQueue.hpp"

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

template <class T>
TrackingQueue<T>::TrackingQueue()
{
	size = 0;
	rrIndex = 0;
}

template <class T>
TrackingQueue<T>::~TrackingQueue()
{
	for (std::map<int, AbstractCameraQueue*>::iterator it = queues.begin(); it != queues.end(); it++) {
		delete it->second;
	}
}

template <class T>
void TrackingQueue<T>::enqueue(std::vector<CameraData> data)
{
	for (std::vector<CameraData>::iterator it = data.begin(); it != data.end(); it++) {
		enqueue(*it);
	}
}

template <class T>
void TrackingQueue<T>::enqueue(CameraData data)
{
	if (queues.count(data.quadcopterId) == 0) {
		queues[data.quadcopterId] = new T(15L * 1000 * 1000 * 1000 / 30,
										  18L * 1000 * 1000 * 1000 / 30,
										  4L * 1000 * 1000 * 1000 / 30);
		ids.push_back(data.quadcopterId);
	}
	
	size++;
	queues[data.quadcopterId]->enqueue(data);
}

template <class T>
size_t TrackingQueue<T>::getSize()
{
	return size;
}

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

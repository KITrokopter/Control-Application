#pragma once

#include "AbstractCameraQueue.hpp"
#include "Graph.hpp"

#include <queue>
#include <map>

/**
 * A camera queue that works round-robin style.
 * That means, that always only one CameraData
 * is returned. The return order is round-robin
 * by camera id, and per camera it is first come
 * first serve.
 *
 * @author Sebastian Schmidt
 */
class RRCameraQueue : public AbstractCameraQueue {
public:
	RRCameraQueue();

	// Implementations of AbstractCameraQueue
	size_t getSize();
	bool dataAvailable();
	std::vector<CameraData> dequeue();

protected:
	// Implementations of AbstractCameraQueue
	void enqueueInternal(CameraData data);

private:
	size_t size;
	size_t rrIndex;
	std::map<int, std::queue<CameraData> > queues;
	std::vector<int> camNos;

	Graph graph;
};
#include "RRCameraQueue.hpp"

#include <ros/console.h>

/**
 * Creates a new RRCameraQueue. The queue will display the
 * size of the per camera queues using a graph with
 * different colors.
 */
RRCameraQueue::RRCameraQueue() : graph(10, "Queue sizes")
{
	rrIndex = 0;
	size = 0;

	std::map<int, cv::Scalar> colors;
	colors[0] = cv::Scalar(0, 255, 0);
	colors[1] = cv::Scalar(0, 255, 255);
	colors[2] = cv::Scalar(255, 0, 255);
	graph.setColors(colors);
}

void RRCameraQueue::enqueueInternal(CameraData data)
{
	if (queues.count(data.camNo) == 0) {
		camNos.push_back(data.camNo);
		queues[data.camNo] = std::queue<CameraData>();
	}

	queues[data.camNo].push(data);
	size++;
}

std::vector<CameraData> RRCameraQueue::dequeue()
{
	if (size == 0) {
		return std::vector<CameraData>();
	}

	int index = rrIndex;
	int loopCount = 0;

	do {
		index = (index + 1) % camNos.size();
		loopCount++;

		if (queues[camNos[index]].size() > 0) {
			break;
		}
	} while (index != rrIndex);

	if (loopCount > camNos.size()) {
		return std::vector<CameraData>();
	} else {
		rrIndex = index;
		CameraData result = queues[camNos[index]].front();
		queues[camNos[index]].pop();
		size--;

		return toVector(result);
	}
}

/**
 * Returns true if there is data to return by the
 * {@link #dequeue()} function, false otherwise.
 *
 * @return True if there is data to be dequeued, false otherwise.
 */
bool RRCameraQueue::dataAvailable()
{
	static int counter = 0;

	if (counter++ % 20 == 0) {
		for (std::map<int, std::queue<CameraData> >::iterator it = queues.begin(); it != queues.end(); it++) {
			graph.nextPoint(it->second.size(), it->first);
		}
	}

	return size > 0;
}

size_t RRCameraQueue::getSize()
{
	return size;
}
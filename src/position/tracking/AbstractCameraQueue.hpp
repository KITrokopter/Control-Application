#pragma once

#include <inttypes.h>
#include <vector>

#include "CameraData.hpp"


class AbstractCameraQueue {
public:
	void enqueue(std::vector<CameraData> data);
	void enqueue(CameraData data);
	
	virtual size_t size() = 0;
	virtual void enqueueInternal(CameraData data) = 0;
	virtual std::vector<CameraData> dequeue() = 0;
};
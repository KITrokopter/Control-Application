#pragma once

#include <inttypes.h>
#include <vector>

#include "CameraData.hpp"


/**
 * An abstract camera queue.
 * The abstract class handles multiple enqueue formats.
 *
 * @author Sebastian Schmidt
 */
class AbstractCameraQueue {
public:
	void enqueue(std::vector<CameraData> data);
	void enqueue(CameraData data);

	/**
	 * Returns the amount of elements in the queue.
	 *
	 * @return The size of the queue.
	 */
	virtual size_t getSize() = 0;

	/**
	 * Returns true if the queue can return data, false
	 * otherwise. Note that not all imeplementations
	 * can implement this method as specified here.
	 *
	 * @return True, if the queue can return data, false otherwise.
	 */
	virtual bool dataAvailable() = 0;

	/**
	 * Removes and returns one or more CameraData from the queue.
	 * The implementation decides what exactly should be returned.
	 *
	 * @return A vector containing CameraData.
	 */
	virtual std::vector<CameraData> dequeue() = 0;

protected:
	std::vector<CameraData> toVector(CameraData data);

	/**
	 * Enqueues the given CameraData.
	 *
	 * @param data The CameraData.
	 */
	virtual void enqueueInternal(CameraData data) = 0;
};
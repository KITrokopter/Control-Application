#pragma once

#include "AbstractCameraQueue.hpp"

#include <list>
#include <set>

/**
 * A camera queue that tries to synchronize the CameraData.
 * It tries to dequeue a set of CameraData containing one
 * CameraData for each camera known at once.
 * That means, that if not all cameras see the quadcopter,
 * the delay induced by this queue will be at least
 * arrivalDelay (Set in the constructor).
 *
 * This queue has some important parameters:
 * <ul>
 * <li>arrivalDelay: A CameraData has to wait at least this time in the queue,
 *                   if it cannot take part in a completely filled group.</li>
 * <li>maxDelay: Entries that stayed longer in the queue than maxDelay will
 *               be dequeued and returned, no matter what.</li>
 * <li>maxGroupInterval: The maximum difference between the oldest and newest
 *                       CameraData creation time in a valid group.</li>
 * <li>ensureCam0: If true, a group is only valid, if it contains CameraData
 *                 from camera 0.</li>
 * </ul>
 *
 * It is ensured that the queue always runs forward. If some data is returned,
 * all data older than that is removed from the queue.
 *
 * The queue uses two time values to ensure that it doesn't create
 * too much latency and that the CameraData sets dequeued are as
 * synchonous as possible. The first is the arrival time, which
 * simply is the time the CameraData was inserted. The second one
 * is the creation time of the CameraData, which is the time the
 * picture that is source of the data was received from the camera.
 *
 * For easy searching for synchronous groups, the queue is always
 * sorted by the CameraData creation time.
 *
 * @author Sebastian Schmidt
 */
class SynchronousCameraQueue : public AbstractCameraQueue {
public:
	SynchronousCameraQueue(long int arrivalDelay, long int maxDelay, long int maxGroupInterval, bool ensureCam0);

	// Implementations of AbstractCameraQueue
	size_t getSize();
	bool dataAvailable();

	std::vector<CameraData> dequeue();

protected:
	// Implementations of AbstractCameraQueue
	void enqueueInternal(CameraData data);

private:
	/**
	 * Pairs a CameraData with its arrival time
	 * in the queue.
	 *
	 * @author Sebastian Schmidt
	 */
	class Bucket {
public:
		CameraData data;
		long int arrivalTime;

		/**
		 * This bucket is smaller than the other, if
		 * this buckets data is older than the other
		 * buckets data.
		 */
		bool operator <(Bucket b)
		{
			return data.time < b.data.time;
		}
	};

	/**
	 * A group of CameraData, containing at most one
	 * CameraData per camera. This is used to
	 * calculate the best return candidate on
	 * dequeue.
	 *
	 * @author Sebastian Schmidt
	 */
	class Group {
public:
		Group();
		Group(std::list<Bucket>::iterator it, long int currentTime, long int arrivalDelay, long int maxDelay,
		      long int maxGroupInterval, long int cameraCount, bool ensureCam0);

		void calculateValue();
		void add(std::list<Bucket>::iterator it);

		bool isValid();
		bool hasWaiting();
		bool wouldInvalidateGroup(std::list<Bucket>::iterator it);
		long int getValue();
		long int getMinTime();
		long int getMaxTime();

		std::list<Bucket>::iterator getYoungest();

		std::vector<CameraData> getData();

private:
		bool canBeValid;
		long int value;
		long int minTime;
		long int maxTime;
		long int currentTime;
		long int arrivalDelay;
		long int maxDelay;
		long int maxGroupInterval;
		long int cameraCount;
		bool ensureCam0;
		std::list<Bucket>::iterator youngest;
		std::list<Bucket>::iterator oldest;
		std::vector<CameraData> data;
	};

	Group searchGroup(std::list<Bucket>::iterator it, long int currentTime, std::list<Bucket>::iterator begin,
	                  std::list<Bucket>::iterator end);
	void cutOffQueue(std::list<Bucket>::iterator it);
	void printQueue();

	long int maxDelay;
	long int arrivalDelay;
	long int maxGroupInterval;
	bool ensureCam0;

	/// Make arrival time unique
	long int lastArrivalTime;

	/// Ensure that tracking does not run backward
	long int minimumPictureTime;

	std::list<Bucket> queue;
	std::set<int> camNos;
};

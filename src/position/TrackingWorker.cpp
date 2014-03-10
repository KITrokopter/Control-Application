#include "TrackingWorker.hpp"

#include <boost/chrono/duration.hpp>
#include <ros/console.h>

TrackingWorker::TrackingWorker(IPositionReceiver *receiver)
{
	assert(receiver != 0);
	
	this->receiver = receiver;
	stop = false;
	
	thread = new boost::thread(boost::bind(&TrackingWorker::run, this));
}

TrackingWorker::~TrackingWorker()
{
	stop = true;
	thread->join();
}

void TrackingWorker::run()
{
	ROS_INFO("Started tracking thread");
	
	bool receivedFirstPosition = false;
	
	while (!stop) {
		CameraData data = dequeue();
		
		if (data.valid) {
			// ROS_DEBUG("Got valid data");
			receivedFirstPosition = true;
			Vector position = tracker.updatePosition(data.cameraVector, data.camNo, data.quadcopterId);
			
			std::vector<Vector> positions;
			std::vector<int> ids;
			std::vector<int> updates;
			positions.push_back(position);
			ids.push_back(data.quadcopterId);
			updates.push_back(1);
			
			if (position.isValid()) {
				receiver->updatePositions(positions, ids, updates);
				ROS_DEBUG("Updated position of quadcopter %d", data.quadcopterId);
			} else {
				ROS_DEBUG("Not enough information to get position of quadcopter %d", data.quadcopterId);
			}
		} else if (receivedFirstPosition == true) {
			// ROS_WARN("Position update buffer is empty!");
		}
	}
	
	ROS_INFO("Stopped tracking thread");
}

void TrackingWorker::updatePosition(Vector cameraVector, int camNo, int quadcopterId)
{
	CameraData data;
	data.cameraVector = cameraVector;
	data.camNo = camNo;
	data.quadcopterId = quadcopterId;
	data.valid = true;
	
	updatePosition(data);
}

void TrackingWorker::updatePosition(CameraData data)
{
	enqueue(data);
	
	ROS_DEBUG("Inserted CameraData for camera %d and quadcopter %d", data.camNo, data.quadcopterId);
}

void TrackingWorker::enqueue(CameraData data)
{
	// ROS_DEBUG("Inserting CameraData");
	
	{
		boost::mutex::scoped_lock lock(positionsMutex);
		positions.push(data);
		
		if (positions.size() > 25) {
			ROS_WARN("Position update buffer is running full (%ld entries). Seems like the position updating can't keep up!", positions.size());
		}
	}
	
	// ROS_DEBUG("Notifying about new CameraData");
	
	positionsEmpty.notify_one();
	
	// ROS_DEBUG("Finished insertion process");
}

CameraData TrackingWorker::dequeue()
{
	// ROS_DEBUG("Getting positions lock");
	
	{
		boost::mutex::scoped_lock lock(positionsMutex);
		
		// ROS_DEBUG("Got positions lock");
		
		if (!dataAvailable()) {
			positionsEmpty.timed_wait(lock, boost::get_system_time() + boost::posix_time::milliseconds(100));
		}
		
		if (dataAvailable()) {
			CameraData data = positions.back();
			positions.pop();
			return data;
		} else {
			CameraData data;
			data.valid = false;
			return data;
		}
	}
	
	// ROS_DEBUG("Released positions lock");
}

bool TrackingWorker::dataAvailable()
{
	return positions.size() > 0;
}

bool TrackingWorker::calibrate(ChessboardData *chessboard, int camNo)
{
	return tracker.calibrate(chessboard, camNo);
}
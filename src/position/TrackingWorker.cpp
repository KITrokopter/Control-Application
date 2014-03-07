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
	while (!stop) {
		CameraData data = dequeue();
		
		if (data.valid) {
			Vector position = tracker.updatePosition(data.cameraVector, data.camNo, data.quadcopterId);
			
			std::vector<Vector> positions;
			std::vector<int> ids;
			std::vector<int> updates;
			positions.push_back(position);
			ids.push_back(data.quadcopterId);
			updates.push_back(1);
			
			if (position.isValid()) {
				receiver->updatePositions(positions, ids, updates);
			} else {
				ROS_DEBUG("Not enough information to get position of quadcopter %d", data.quadcopterId);
			}
		}
	}
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
}

void TrackingWorker::enqueue(CameraData data)
{
	{
		boost::mutex::scoped_lock lock(positionsMutex);
		positions.push(data);
		
		if (positions.size() > 25) {
			ROS_WARN("Position update buffer is running full (%ld entries). Seems like the position updating can't keep up!", positions.size());
		}
	}
	
	positionsEmpty.notify_one();
}

CameraData TrackingWorker::dequeue()
{
	{
		boost::mutex::scoped_lock lock(positionsMutex);
		
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
}

bool TrackingWorker::dataAvailable()
{
	return positions.size() > 0;
}

bool TrackingWorker::calibrate(ChessboardData *chessboard, int camNo)
{
	return tracker.calibrate(chessboard, camNo);
}
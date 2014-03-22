#include "TrackingWorker.hpp"

#include <boost/chrono/duration.hpp>
#include <ros/console.h>
#include <opencv2/core/core.hpp>
#include <map>

TrackingWorker::TrackingWorker(IPositionReceiver *receiver) : errorGraph(200, "Difference")
{
	assert(receiver != 0);
	
	this->receiver = receiver;
	stop = false;
	
	std::map<int, cv::Scalar> colors;
	colors[0] = cv::Scalar(0, 255, 0);
	colors[1] = cv::Scalar(0, 255, 255);
	colors[2] = cv::Scalar(255, 0, 255);
	errorGraph.setColors(colors);
	
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
		std::vector<CameraData> data = dequeue();
		
		if (data.size() > 0) {
			if (!receivedFirstPosition) {
				receivedFirstPosition = true;
				ROS_INFO("Found quadcopter %d", data[0].quadcopterId);
			}
			
			Vector position = tracker.updatePosition(data);
			
			for (size_t i = 0; i < data.size(); i++) {
				errorGraph.nextPoint(tracker.getDistance(), data[i].camNo);
			}
			
			std::vector<Vector> positions;
			std::vector<int> ids;
			std::vector<int> updates;
			positions.push_back(position);
			ids.push_back(data[0].quadcopterId);
			updates.push_back(1);
			
			if (position.isValid()) {
				receiver->updatePositions(positions, ids, updates);
			}
			
			// ROS_DEBUG("Updating position of quadcopter %d took %.3f ms", data.quadcopterId, duration);
		} else if (receivedFirstPosition) {
			ROS_WARN("Position update buffer is empty!");
		}
	}
	
	ROS_INFO("Stopped tracking thread");
}

void TrackingWorker::updatePosition(Vector cameraVector, int camNo, int quadcopterId, long int time)
{
	CameraData data;
	data.cameraVector = cameraVector;
	data.camNo = camNo;
	data.quadcopterId = quadcopterId;
	data.valid = true;
	data.time = time;
	
	updatePosition(data);
}

void TrackingWorker::updatePosition(CameraData data)
{
	enqueue(data);
}

void TrackingWorker::enqueue(CameraData data)
{
	boost::mutex::scoped_lock lock(queueMutex);
	
	queue.enqueue(data);
	
	queueEmpty.notify_one();
}

std::vector<CameraData> TrackingWorker::dequeue()
{
	boost::mutex::scoped_lock lock(queueMutex);
	
	if (!dataAvailable()) {
		queueEmpty.timed_wait(lock, boost::get_system_time() + boost::posix_time::milliseconds(100));
	}
	
	if (dataAvailable()) {
		return queue.dequeue();
	} else {
		return std::vector<CameraData>();
	}
}

bool TrackingWorker::dataAvailable()
{
	return queue.dataAvailable();
}

bool TrackingWorker::calibrate(ChessboardData *chessboard, int camNo)
{
	return tracker.calibrate(chessboard, camNo);
}

Vector TrackingWorker::getCameraPosition(int camNo)
{
	return tracker.getPosition(camNo);
}

cv::Mat TrackingWorker::getIntrinsicsMatrix(int camNo)
{
	return tracker.getIntrinsicsMatrix(camNo);
}

cv::Mat TrackingWorker::getDistortionCoefficients(int camNo)
{
	return tracker.getDistortionCoefficients(camNo);
}

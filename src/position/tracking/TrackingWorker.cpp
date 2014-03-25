#include "TrackingWorker.hpp"

#include <boost/chrono/duration.hpp>
#include <ros/console.h>
#include <opencv2/core/core.hpp>
#include <map>

#include "../../matlab/profiling.hpp"

TrackingWorker::TrackingWorker(IPositionReceiver *receiver) : errorGraph(100, "Difference")
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
	int emptyCount = 0;
	
	while (!stop) {
		std::vector<CameraData> data = dequeue();
		
		if (data.size() > 0) {
			emptyCount = 0;
			
			if (!receivedFirstPosition) {
				receivedFirstPosition = true;
				ROS_INFO("Found quadcopter %d", data[0].quadcopterId);
			}
			
			// ROS_DEBUG("Got info from camera %d: [%.2f, %.2f, %.2f]", data[0].camNo, data[0].cameraVector.getV1(), data[0].cameraVector.getV2(), data[0].cameraVector.getV3());
			
			Vector position = tracker.updatePosition(data);
			
			for (size_t i = 0; i < data.size(); i++) {
				errorGraph.nextPoint(tracker.getDistance(), data[i].camNo);
			}
			
			
			if (position.isValid()) {
				// Invert x axis for controller
				position.setV1(-position.getV1());
				
				std::vector<Vector> positions;
				std::vector<int> ids;
				std::vector<int> updates;
				positions.push_back(position);
				ids.push_back(data[0].quadcopterId);
				updates.push_back(1);
				
				receiver->updatePositions(positions, ids, updates);
			}
		} else if (receivedFirstPosition) {
			emptyCount++;
			
			if (emptyCount == 50) {
				// TODO uncomment
				// ROS_WARN("Position update buffer is empty!");
				emptyCount = 0;
			}
			
			boost::mutex::scoped_lock lock(queueMutex);
			queueEmpty.wait(lock);
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
	
	queueEmpty.notify_all();
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

void TrackingWorker::updateTrackingArea()
{
	receiver->setTrackingArea(tracker.getTrackingArea());
}

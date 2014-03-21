#include "TrackingWorker.hpp"

#include <boost/chrono/duration.hpp>
#include <ros/console.h>
#include <opencv2/core/core.hpp>
#include <map>

#include "../matlab/profiling.hpp"

TrackingWorker::TrackingWorker(IPositionReceiver *receiver) : tracker(true), errorGraph(200, "Difference"), positions(50)
{
	assert(receiver != 0);
	
	this->receiver = receiver;
	stop = false;
	rrCounter = 0;
	maxCamNo = 0;
	bufferSize = 0;
	minUpdateCount = 2;
	
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
		CameraData data = dequeue();
		
		if (data.valid) {
			// ROS_DEBUG("Got valid data");
			receivedFirstPosition = true;
			
			long int startTime = getNanoTime();
			Vector position = tracker.updatePosition(data.cameraVector, data.camNo, data.quadcopterId);
			errorGraph.nextPoint(tracker.getDistance(), data.camNo);
			double duration = getNanoTime() - startTime;
			duration /= 1e6;
			
			std::vector<Vector> positions;
			std::vector<int> ids;
			std::vector<int> updates;
			positions.push_back(position);
			ids.push_back(data.quadcopterId);
			updates.push_back(1);
			
			if (position.isValid()) {
				receiver->updatePositions(positions, ids, updates);
			} else {
				// ROS_DEBUG("Not enough information to get position of quadcopter %d", data.quadcopterId);
			}
			
			// ROS_DEBUG("Updating position of quadcopter %d took %.3f ms", data.quadcopterId, duration);
		} else if (receivedFirstPosition) {
			ROS_WARN("Position update buffer is empty!");
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
	// ROS_DEBUG("updatePosition: called");
	
	enqueue(data);
	
	// ROS_DEBUG("updatePosition: Inserted CameraData for camera %d and quadcopter %d", data.camNo, data.quadcopterId);
}

void TrackingWorker::enqueue(CameraData data)
{
	// ROS_DEBUG("enqueue: Inserting CameraData");
	
	{
		boost::mutex::scoped_lock lock(positionsMutex);
		// ROS_DEBUG("enqueue: Got positions lock");
		
		if (data.camNo + 1 > maxCamNo) {
			maxCamNo = data.camNo + 1;
			ROS_DEBUG("Increased max cam no to %d", maxCamNo);
		}
		
		positions[data.camNo].push(data);
		
		bufferSize++;
		
		if (bufferSize > 25) {
			ROS_WARN("Position update buffer is running full (%ld entries). Seems like the position updating can't keep up! Dropping 15 entries.", positions.size());
			
			int deleted = 0;
			int index = rrCounter;
			
			while (deleted < 15) {
				if (positions[index].size() > 0) {
					positions[index].pop();
					deleted++;
					bufferSize--;
				}
				
				index = (index + 1) % maxCamNo;
			}
		}
	}
	
	// ROS_DEBUG("enqueue: Released positions lock");
	// ROS_DEBUG("enqueue: Notifying about new CameraData");
	
	positionsEmpty.notify_one();
	
	// ROS_DEBUG("enqueue: Finished insertion process");
}

CameraData TrackingWorker::dequeue()
{
	// ROS_DEBUG("dequeue: Getting positions lock");
	
	{
		boost::mutex::scoped_lock lock(positionsMutex);
		
		// ROS_DEBUG("dequeue: Got positions lock");
		
		if (!dataAvailable()) {
			ROS_DEBUG("Not enough data available, waiting...");
			positionsEmpty.timed_wait(lock, boost::get_system_time() + boost::posix_time::milliseconds(100));
			ROS_DEBUG("Waited");
		}
		
		if (dataAvailable()) {
			do {
				rrCounter++;
				rrCounter %= maxCamNo;
			} while (positions[rrCounter].size() == 0);
			
			CameraData data = positions[rrCounter].front();
			positions[rrCounter].pop();
			bufferSize--;
			
			return data;
		} else {
			CameraData data;
			data.valid = false;
			
			// ROS_DEBUG("dequeue: Released positions lock");
			return data;
		}
	}
}

bool TrackingWorker::dataAvailable()
{
	for (int i = 0; i < maxCamNo; i++) {
		if (positions[i].size() > 0) {
			ROS_DEBUG("Found %d data for camno %d", positions[i].size(), i);
			return true;
		}
	}
	
	return false;
}

bool TrackingWorker::haveEnoughData(int count)
{
	int data = 0;
	
	for (int i = 0; i < maxCamNo; i++) {
		if (positions[i].size() > 0) {
			data++;
			
			if (data >= count) {
				return true;
			}
		}
	}
		
	return false;
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

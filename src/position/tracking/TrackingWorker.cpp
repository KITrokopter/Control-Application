#include "TrackingWorker.hpp"

#include <boost/chrono/duration.hpp>
#include <opencv2/core/core.hpp>
#include <sstream>

#include "../../matlab/profiling.hpp"
#include "control_application/quadcopter_position.h"

/**
 * Constructs a new tracking worker and starts the working thread.
 * 
 * @param receiver The object the calculated positions should be passed to.
 */
TrackingWorker::TrackingWorker(IPositionReceiver *receiver) : errorGraph(100, "Difference"), latencyGraph(500, "Latency")
{
	assert(receiver != 0);
	
	this->receiver = receiver;
	stop = false;
	
	std::map<int, cv::Scalar> colors;
	colors[0] = cv::Scalar(0, 255, 0);
	colors[1] = cv::Scalar(0, 255, 255);
	colors[2] = cv::Scalar(255, 0, 255);
	errorGraph.setColors(colors);
	latencyGraph.setColors(colors);
	
	thread = new boost::thread(boost::bind(&TrackingWorker::run, this));
}

/**
 * Destroys the tracking worker and joins the worker thread.
 */
TrackingWorker::~TrackingWorker()
{
	stop = true;
	thread->join();
}

/**
 * Is run by the worker thread. Checks if enough camera data is available.
 * If there is enough, a new position is calculated and sent to the position receiver.
 */
void TrackingWorker::run()
{
	ROS_INFO("Started tracking thread");
	
	bool receivedFirstPosition = false;
	int emptyCount = 0;
	
	while (!stop) {
		std::vector<CameraData> data = dequeue();
		
		if (data.size() > 1) {
			emptyCount = 0;
			
			if (!receivedFirstPosition) {
				receivedFirstPosition = true;
				ROS_INFO("Found quadcopter %d", data[0].quadcopterId);
			}
			
			// ROS_DEBUG("Got info from camera %d: [%.2f, %.2f, %.2f]", data[0].camNo, data[0].cameraVector.getV1(), data[0].cameraVector.getV2(), data[0].cameraVector.getV3());
			
			double latency = 0;
			double currentTime = getNanoTime();
			
			for (std::vector<CameraData>::iterator it = data.begin(); it != data.end(); it++) {
				double newLatency = currentTime - it->time;
				
				if (newLatency > latency) {
					latency = newLatency;
				}
			}
			
			latencyGraph.nextPoint(latency / 1e6, 0);
			ROS_DEBUG("POSITION_MODULE: Latency is %.3fms", latency / 1e6);
			
			Vector position = tracker.updatePosition(data);
			
			for (size_t i = 0; i < data.size(); i++) {
				errorGraph.nextPoint(tracker.getDistance(), data[i].camNo);
			}
			
			
			if (position.isValid()) {
				// Invert x axis for controller. Uncomment if necessary, but only if you really thought about it.
				// position.setV1(-position.getV1());
				
				sendPosition(position, data[0].quadcopterId);
				
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
				ROS_WARN("POSITION_MODULE: Position update buffer is empty!");
				emptyCount = 0;
			}
			
			boost::mutex::scoped_lock lock(queueMutex);
			queueEmpty.wait(lock);
		}
	}
	
	ROS_INFO("Stopped tracking thread");
}

/**
 * Schedules camera data to be processed.
 * 
 * @param cameraVector The direction the camera sees the crazyflie in.
 * @param camNo The number of the camera.
 * @param quadcopterId The id of the quadcopter.
 * @param time The time the data was measured.
 */
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

/**
 * Schedules the given camera data to be processed.
 * 
 * @param data The camera data.
 */
void TrackingWorker::updatePosition(CameraData data)
{
	enqueue(data);
}

/**
 * Enqueues a camera data object into the queue.
 * 
 * @param data The data to be enqueued.
 */
void TrackingWorker::enqueue(CameraData data)
{
	boost::mutex::scoped_lock lock(queueMutex);
	
	queue.enqueue(data);
	
	queueEmpty.notify_all();
}

/**
 * Dequeues camera data from one or more cameras.
 * 
 * @return Data from one or more cameras.
 */
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

/**
 * Returns true if there is data in the queue that can be returned.
 * If it returns false, it is does not always mean that there is really nothing to return.
 * Classes implementing this should ensure that if there is good data available, true is returned.
 * 
 * @return True if data is available, false if it can not be ensured that data is available.
 */
bool TrackingWorker::dataAvailable()
{
	return queue.dataAvailable();
}

/**
 * Passes the calibration request to the underlying position calculator.
 * 
 * @param chessboard The properties of the chessboard for calibration.
 * @param camNo The amount of cameras that are calibrated.
 * @return True if the calibration was successful, false otherwise.
 */
bool TrackingWorker::calibrate(ChessboardData *chessboard, int cameraAmount)
{
	return tracker.calibrate(chessboard, cameraAmount);
}

/**
 * Returns the position of the camera with the given number.
 * 
 * @param camNo The number of the camera.
 * @return The position of the camera.
 */
Vector TrackingWorker::getCameraPosition(int camNo)
{
	return tracker.getPosition(camNo);
}

/**
 * Returns the rotation matrix of the camera with the given number.
 * 
 * @param camNo The number of the camera.
 * @return The rotation matrix of the camera.
 */
Matrix TrackingWorker::getRotationMatrix(int camNo)
{
	return tracker.getRotationMatrix(camNo);
}

/**
 * Returns the intrinsics matrix of the camera with the given number.<br />
 * (This is part of the single camera calibration result.)
 * 
 * @param camNo The number of the camera.
 * @return The intrinsics matrix of the camera.
 */
cv::Mat TrackingWorker::getIntrinsicsMatrix(int camNo)
{
	return tracker.getIntrinsicsMatrix(camNo);
}

/**
 * Returns the distortion coefficients of the camera with the given number.<br />
 * (This is part of the single camera calibration result.)
 * 
 * @param camNo The number of the camera.
 * @return The distortion coefficients of the camera.
 */
cv::Mat TrackingWorker::getDistortionCoefficients(int camNo)
{
	return tracker.getDistortionCoefficients(camNo);
}

/**
 * Passes the tracking area to the position receiver.
 */
void TrackingWorker::updateTrackingArea()
{
	receiver->setTrackingArea(tracker.getTrackingArea());
}

/**
 * Sends a position vector to the quadcopter_position_{id} topic.
 * 
 * @param position The position of the quadcopter.
 * @param quadcopterId The id of the quadcopter.
 */
void TrackingWorker::sendPosition(Vector position, int quadcopterId)
{
	if (quadcopterPositionPublishers.count(quadcopterId) == 0) {
		std::stringstream name;
		name << "quadcopter_position_" << quadcopterId;
		
		ros::NodeHandle n;
		quadcopterPositionPublishers[quadcopterId] = n.advertise<control_application::quadcopter_position>(name.str(), 4);
	}
	
	control_application::quadcopter_position msg;
	msg.x = position.getV1();
	msg.y = position.getV2();
	msg.z = position.getV3();
	quadcopterPositionPublishers[quadcopterId].publish(msg);
}

#pragma once

#include <boost/thread.hpp>
#include <queue>
#include <map>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition_variable.hpp>

#include "../IPositionReceiver.hpp"
#include "../../matlab/Vector.h"
#include "../../matlab/Position.h"
#include "../../controller/Mutex.hpp"

#include "Graph.hpp"
#include "CameraData.hpp"
#include "RRCameraQueue.hpp"
#include "TrackingQueue.hpp"

class TrackingWorker {
private:
	boost::thread *thread;
	IPositionReceiver *receiver;
	Position tracker;
	volatile bool stop;

	TrackingQueue<RRCameraQueue> queue;
	boost::mutex queueMutex;
	boost::condition_variable queueEmpty;
	
	// Graphing
	Graph errorGraph;
	
	void run();
	
	void enqueue(CameraData data);
	std::vector<CameraData> dequeue();
	bool dataAvailable();
	bool haveEnoughData(int count);
public:
	TrackingWorker(IPositionReceiver *receiver);
	~TrackingWorker();
	
	void updatePosition(Vector cameraVector, int camNo, int quadcopterId, long int time);
	void updatePosition(CameraData cameraData);
	
	bool calibrate(ChessboardData *chessboard, int camNo);
	Vector getCameraPosition(int camNo);
	cv::Mat getIntrinsicsMatrix(int camNo);
	cv::Mat getDistortionCoefficients(int camNo);
	void updateTrackingArea();
};

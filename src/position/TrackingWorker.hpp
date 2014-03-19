#pragma once

#include <boost/thread.hpp>
#include <queue>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition_variable.hpp>

#include "IPositionReceiver.hpp"
#include "../matlab/Vector.h"
#include "../matlab/Position.h"
#include "../controller/Mutex.hpp"

#include "Graph.hpp"

typedef struct {
	Vector cameraVector;
	int camNo;
	int quadcopterId;
	bool valid;
} CameraData;

class TrackingWorker {
private:
	boost::thread *thread;
	IPositionReceiver *receiver;
	Position tracker;
	volatile bool stop;

	std::queue<CameraData> positions;
	boost::mutex positionsMutex;
	boost::condition_variable positionsEmpty;
	
	// Graphing
	Graph errorGraph;
	
	void run();
	
	void enqueue(CameraData data);
	CameraData dequeue();
	bool dataAvailable();
public:
	TrackingWorker(IPositionReceiver *receiver);
	~TrackingWorker();
	
	void updatePosition(Vector cameraVector, int camNo, int quadcopterId);
	void updatePosition(CameraData cameraData);
	
	bool calibrate(ChessboardData *chessboard, int camNo);
	Vector getCameraPosition(int camNo);
	cv::Mat getIntrinsicsMatrix(int camNo);
	cv::Mat getDistortionCoefficients(int camNo);
};
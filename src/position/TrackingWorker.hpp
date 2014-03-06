#pragma once

#include <boost/thread.h>
#include <queue>
#include <mutex>              // std::mutex, std::unique_lock
#include <condition_variable> // std::condition_variable

#include "IPositionReceiver.hpp"
#include "../matlab/Vector.h"
#include "../matlab/Position.h"
#include "../controller/Mutex.hpp"

typedef struct {
	Vector cameraVector;
	int camNo;
	int quadcopterId;
} CameraData;

class TrackingWorker {
private:
	boost::thread *thread;
	IPositionReceiver *receiver;
	Position tracker;
	volatile bool stop;

	std::queue<CameraData> positions;
	std::mutex mtx;
	std::condition_variable cv;
	
	void run();
	
	void enqueue(CameraData data);
	CameraData dequeue();
	bool dataAvailable();
public:
	TrackingWorker(IPositionReceiver *receiver);
	~TrackingWorker();
	
	void updatePosition(Vector cameraVector, int camNo, int quadcopterId);
	void updatePosition(CameraData cameraData);
};
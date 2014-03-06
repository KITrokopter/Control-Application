#include "TrackingWorker.hpp"

TrackingWorker::TrackingWorker(IPositionReceiver *receiver)
{
	this->receiver = receiver;
	stop = false;
	
	thread = new boost::thread(boost::bind(&TrackingWorker::run, this));
}

TrackingWorker::~TrackingWorker()
{
	thread->join();
}

void TrackingWorker::enqueue(CameraData data)
{
	std::unique_lock<std::mutex> lck(mtx);
	positions.push_back(data);
	cv.notify_one();
}

CameraData TrackingWorker::dequeue()
{
	std::unique_lock<std::mutex> lck(mtx);
	cv.wait(lck,shipment_available);
	return positions.pop_front();
}

bool TrackingWorker::dataAvailable()
{
	return positions.size() > 0;
}
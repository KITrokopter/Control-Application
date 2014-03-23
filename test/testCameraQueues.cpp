#include <iostream>
#include <string>
#include <sstream>

#include <ros/ros.h>

#define DEBUG_TIME
#include "../src/matlab/profiling.hpp"
#include "../src/position/tracking/SynchronousCameraQueue.hpp"

using namespace std;

string toString(long int i)
{
	stringstream ss;
	ss << i;
	return ss.str();
}

void insertData(SynchronousCameraQueue *q, int camNo, long int time)
{
	
	setNanoTime(time + 4);
	CameraData data;
	data.valid = true;
	data.camNo = camNo;
	data.quadcopterId = 42;
	data.time = time;
	data.cameraVector = Vector(camNo, time, -1);
	q->enqueue(data);
}

string testGrouping()
{
	setNanoTime(0);
	
	SynchronousCameraQueue q(10, 20, 10);
	
	insertData(&q, 0, 2);
	insertData(&q, 1, 1);
	insertData(&q, 2, 5);
	insertData(&q, 0, 14);
	insertData(&q, 0, 30);
	insertData(&q, 1, 12);
	insertData(&q, 0, 39);
	insertData(&q, 1, 22);
	insertData(&q, 1, 31);
	insertData(&q, 2, 17);
	insertData(&q, 2, 29);
	insertData(&q, 2, 36);
	insertData(&q, 1, 41);
	
	setNanoTime(45);
	
	vector<CameraData> result = q.dequeue();
	
	/*for (vector<CameraData>::iterator it = result.begin(); it != result.end(); it++) {
		cout << "Result cam/time: " << it->camNo << "/" << it->time << endl;
	}*/
	
	if (result.size() != 3) {
		return "1: Result size should be 3, but is " + toString(result.size());
	}
	
	if (q.getSize() != 3) {
		return "2: Queue size should be 3, but is " + toString(q.getSize());
	}
	
	return "";
}

string testDataAvailable()
{
	setNanoTime(0);
	
	SynchronousCameraQueue q(10, 20, 10);
	
	CameraData data;
	data.valid = true;
	data.camNo = 0;
	data.quadcopterId = 23;
	data.time = 1;
	data.cameraVector = Vector(1, 1, 1);
	q.enqueue(data);
	
	data.camNo = 1;
	data.cameraVector = Vector(2, 2, 2);
	q.enqueue(data);
	
	data.camNo = 2;
	data.cameraVector = Vector(3, 3, 3);
	q.enqueue(data);
	
	q.dequeue();
	
	if (q.getSize() != 0) {
		return "4: Size should be 0, but is " + toString(q.getSize());
	}
	
	data.time = 2;
	q.enqueue(data);
	
	if (q.dataAvailable()) {
		return "1: There should not be data available";
	}
	
	setNanoTime(12);
	
	if (q.dataAvailable()) {
		return "2: There should not be data available";
	}
	
	setNanoTime(27);
	
	if (!q.dataAvailable()) {
		return "3: There should be data available";
	}
	
	return "";
}

string testDequeue()
{
	setNanoTime(0);
	
	SynchronousCameraQueue q(10, 20, 10);
	
	CameraData data;
	data.valid = true;
	data.camNo = 0;
	data.quadcopterId = 23;
	data.time = 2;
	data.cameraVector = Vector(1, 1, 1);
	q.enqueue(data);
	
	data.camNo = 1;
	data.cameraVector = Vector(2, 2, 2);
	q.enqueue(data);
	
	data.camNo = 2;
	data.cameraVector = Vector(3, 3, 3);
	q.enqueue(data);
	
	vector<CameraData> result = q.dequeue();
	
	if (result.size() != 3) {
		return "1: Result size should be 3, but is " + toString(result.size());
	}
	
	if (q.getSize() != 0) {
		return "2: Queue size should be 0, but is " + toString(q.getSize());
	}
	
	return "";
}

string testEnqueue()
{
	setNanoTime(0);
	
	SynchronousCameraQueue q(10, 20, 10);
	
	CameraData data;
	data.valid = true;
	data.camNo = 0;
	data.quadcopterId = 23;
	data.time = 3;
	data.cameraVector = Vector(1, 1, 1);
	q.enqueue(data);
	
	data.camNo = 1;
	data.cameraVector = Vector(2, 2, 2);
	q.enqueue(data);
	
	data.camNo = 2;
	data.cameraVector = Vector(3, 3, 3);
	q.enqueue(data);
	
	if (q.getSize() != 3) {
		return "1: Size should be 3, but is " + toString(q.getSize());
	}
	
	return "";
}

void test(string name, string (*f)())
{
	cout << "EXECUTING: " << name << " ... ";
	
	string result = f();
	
	if (result.length() == 0) {
		cout << "success" << endl;
	} else {
		cout << "FAILED" << endl;
		cout << result << endl;
	}
}

int main()
{
	cout << "=====================================================================";
	cout << "=====================================================================";
	cout << endl;
	
	// Enable debug level logging.
	if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
		ros::console::notifyLoggerLevelsChanged();
	}
	
	test("TestEnqueue", testEnqueue);
	test("TestDequeue", testDequeue);
	test("TestDataAvailable", testDataAvailable);
	test("TestGrouping", testGrouping);
	
	cout << "=====================================================================";
	cout << "=====================================================================";
	cout << endl;
}
#include <iostream>
#include <string>
#include <sstream>

#include <ros/ros.h>

#define DEBUG_TIME
#include "../src/matlab/profiling.hpp"
#include "../src/position/tracking/SynchronousCameraQueue.hpp"

using namespace std;

/**
 * Converts an int to string.
 *
 * @param i The integer.
 * @return i as string.
 */
string toString(long int i)
{
	stringstream ss;
	ss << i;
	return ss.str();
}

/**
 * Inserts data into the camera queue, using the given parameters and some
 * default values.
 *
 * @param q The queue to insert into.
 * @param camNo The number of the camera.
 * @param time The timestamp for the generated data.
 */
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

/**
 * A test case to check if the samples are correctly grouped.
 *
 * @return An empty string if the test was successful, or the error otherwise.
 */
string testGrouping()
{
	setNanoTime(0);

	SynchronousCameraQueue q(10, 20, 10, false);

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

	/*for (vector<CameraData>::iterator it = result.begin(); it != result.end();
	 * it++) {
	 *     cout << "Result cam/time: " << it->camNo << "/" << it->time << endl;
	 *    }*/

	if (result.size() != 3) {
		return "1: Result size should be 3, but is " + toString(result.size());
	}

	if (q.getSize() != 3) {
		return "2: Queue size should be 3, but is " + toString(q.getSize());
	}

	for (vector<CameraData>::iterator it = result.begin(); it != result.end(); it++) {
		switch (it->camNo) {
		case 0:
			if (it->time != 30) {
				return "3: Time should be 30, but was " + toString(it->time);
			}

			break;

		case 1:
			if (it->time != 31) {
				return "4: Time should be 31, but was " + toString(it->time);
			}

			break;

		case 2:
			if (it->time != 29) {
				return "5: Time should be 29, but was " + toString(it->time);
			}

			break;
		}
	}

	return "";
}

/**
 * A test case to check if the dataAvailable() function works correctly.
 *
 * @return An empty string if the test was successful, or the error otherwise.
 */
string testDataAvailable()
{
	setNanoTime(0);

	SynchronousCameraQueue q(10, 20, 10, false);

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

/**
 * A test case to check if the dequeue function works correctly.
 *
 * @return An empty string if the test was successful, or the error otherwise.
 */
string testDequeue()
{
	setNanoTime(0);

	SynchronousCameraQueue q(10, 20, 10, false);

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

/**
 * A test case to check if enqueueing works correctly.
 *
 * @return An empty string if the test was successful, or the error otherwise.
 */
string testEnqueue()
{
	setNanoTime(0);

	SynchronousCameraQueue q(10, 20, 10, false);

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

/**
 * Executes a test method.
 *
 * @param name The name of the test.
 * @param f The test method to call.
 */
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

/**
 * Executes some test methods.
 *
 * @return 0.
 */
int main()
{
	cout << "=====================================================================";
	cout << "=====================================================================";
	cout << endl;

	// Enable debug level logging.
	if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
		ros::console::notifyLoggerLevelsChanged();
	}

	test("TestEnqueue", testEnqueue);
	test("TestDequeue", testDequeue);
	test("TestDataAvailable", testDataAvailable);
	test("TestGrouping", testGrouping);

	cout << "=====================================================================";
	cout << "=====================================================================";
	cout << endl;

	return 0;
}
#include <iostream>
#include <string>

#define DEBUG_TIME
#include "../src/matlab/profiling.hpp"
#include "../src/position/tracking/SynchronousCameraQueue.hpp"

using namespace std;

string testDataAvailable()
{
	setNanoTime(0);
	
	SynchronousCameraQueue q(10, 20, 10);
	
	CameraData data;
	data.valid = true;
	data.camNo = 0;
	data.quadcopterId = 23;
	data.time = 0;
	data.cameraVector = Vector(1, 1, 1);
	q.enqueue(data);
	
	data.camNo = 1;
	data.cameraVector = Vector(2, 2, 2);
	q.enqueue(data);
	
	data.camNo = 2;
	data.cameraVector = Vector(3, 3, 3);
	q.enqueue(data);
	
	q.dequeue();
	q.enqueue(data);
	
	if (q.dataAvailable()) {
		return "1: There should not be data available";
	}
	
	setNanoTime(12);
	
	if (q.dataAvailable()) {
		return "2: There should not be data available";
	}
	
	setNanoTime(23);
	
	if (!q.dataAvailable()) {
		return "3: There should be data available";
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
	
	test("TestDataAvailable", testDataAvailable);
	
	cout << "=====================================================================";
	cout << "=====================================================================";
	cout << endl;
}
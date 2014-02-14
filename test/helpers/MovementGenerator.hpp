#pragma once

#include "../../src/position/IPositionReceiver.hpp"
#include "../../src/matlab/Vector.h"
#include <inttypes.h>
#include <vector>

class MovementGenerator {
private:
	IPositionReceiver *receiver;
	
	std::vector<Vector> from;
	std::vector<Vector> to;
	
	double errorRate;
	double errorSize;
	double defaultErrorSize;
	int quadcopterCount;
	int steps;
	double stepsPerSecond;
	
	double interpolate(double a, double b, double alpha);
	
	/// Generates a random number between 0 and 1.
	double random();
	
	/// Generates a random number between -size and size
	double randomError(double size);
	
	/// Returns the current time in microseconds
	int64_t getNanoseconds();
	
	/// Sleeps for the given amount of nanoseconds
	void sleep(int64_t nanos);
	
public:
	/// Dumb constructor, would need setter to be useful.
	MovementGenerator(IPositionReceiver *receiver);
	MovementGenerator(IPositionReceiver *receiver, std::vector<Vector> from, std::vector<Vector> to, double errorRate, double errorSize, double defaultErrorSize, int steps, double stepsPerSecond);
	
	/// Runs the generator synchronous (This method blocks until it finished)
	void run();
};
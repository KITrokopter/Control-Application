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
	
	/**
	 * Creates a new MovementGenerator object.
	 * The quadcopters in the from and to vectors have the position in the vector as id.<br />
	 * Example: The quadcopter flying from from[2] to to[2] has id 2.
	 * 
	 * @param receiver The position receiver to send the position data to.
	 * @param from The start positions of the quadcopters.
	 * @param to The end positions of the quadcopters.
	 * @param errorRate The probability for a random error.
	 * @param errorSize The maximum size of a random error. A random error ranges from -errorSize to errorSize and is added to the position.
	 *                  For every component of the position vector a new random number is generated, but the random error is applied to
	 *                  either all components of the position vector or none.
	 * @param defaultErrorSize The error that is always applied.
	 * @param steps The amount of steps to use for moving from the start to the end position.
	 * @param stepsPerSecond The amount of steps to calculate per second.
	 */
	MovementGenerator(IPositionReceiver *receiver, std::vector<Vector> from, std::vector<Vector> to, double errorRate, double errorSize, double defaultErrorSize, int steps, double stepsPerSecond);
	
	/// Runs the generator synchronous (This method blocks until it finished)
	void run();
};
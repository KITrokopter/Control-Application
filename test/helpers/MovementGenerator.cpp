#include "MovementGenerator.hpp"

#include <cstdlib>
#include <sys/time.h>
#include <unistd.h>
#include <stdio.h>
#include <iostream>
#include <sstream>
#include <string>
#include <algorithm>

MovementGenerator::MovementGenerator(IPositionReceiver *receiver, std::vector<Vector> from, std::vector<Vector> to, double errorRate, double errorSize, double defaultErrorSize, int steps, double stepsPerSecond)
{
	this->receiver = receiver;
	this->from = from;
	this->to = to;
	this->errorRate = errorRate;
	this->errorSize = errorSize;
	this->defaultErrorSize = defaultErrorSize;
	this->quadcopterCount = std::min(from.size(), to.size());
	this->steps = steps;
	this->stepsPerSecond = stepsPerSecond;
}

double MovementGenerator::interpolate(double a, double b, double alpha)
{
	return a * (1 - alpha) + b * alpha;
}

double MovementGenerator::random()
{
	double random = rand();
	
	random = random / RAND_MAX;
	
	return random;
}

double MovementGenerator::randomError(double size)
{
	double result = random();
	result -= 0.5;
	result *= 2 * size;
	return result;
}

int64_t MovementGenerator::getNanoseconds()
{
	struct timespec t;
	clock_gettime(CLOCK_REALTIME, &t);
	return t.tv_sec * 1000000000 + t.tv_nsec;
}

void MovementGenerator::sleep(int64_t nanos)
{
	struct timespec t;
	t.tv_sec = nanos / 1000000000;
	t.tv_nsec = nanos % 1000000000;
	nanosleep(&t, 0);
}

void MovementGenerator::run()
{
	// Seed random
	time_t rawTime = time(0);
	std::srand(rawTime);
	
	std::cout << "Running MovementGenerator" << std::endl;
	
	// Target time for one loop in us
	int64_t targetTime = 1000 * 1000 * 1000 / stepsPerSecond;
	int64_t startTime = getNanoseconds();
	
	for (int i = 0; i <= steps; i++)
	{
		double alpha = ((double) i) / steps;
		
		std::vector<Vector> positions(quadcopterCount);
		std::vector<int> ids(quadcopterCount);
		std::vector<int> updates(quadcopterCount);
		std::vector<bool> error(quadcopterCount);
		
		for (int j = 0; j < quadcopterCount; j++) {
			Vector result;
			
			result.setV1(interpolate(from[j].getV1(), to[j].getV1(), alpha));
			result.setV2(interpolate(from[j].getV2(), to[j].getV2(), alpha));
			result.setV3(interpolate(from[j].getV3(), to[j].getV3(), alpha));
			
			// Create default error
			result.setV1(result.getV1() + randomError(defaultErrorSize));
			result.setV2(result.getV2() + randomError(defaultErrorSize));
			result.setV3(result.getV3() + randomError(defaultErrorSize));
			
			// Create random error
			if (random() < errorRate) {
				result.setV1(result.getV1() + randomError(errorSize));
				result.setV2(result.getV2() + randomError(errorSize));
				result.setV3(result.getV3() + randomError(errorSize));
				
				error[j] = true;
			} else {
				error[j] = false;
			}
			
			positions[j] = result;
			ids[j] = j;
			updates[j] = 1;
		}
		
		int64_t currentTime = getNanoseconds();
		receiver->updatePositions(positions, ids, updates);
		
		std::cout << "Sent vectors at " << currentTime << std::endl;
		
		for (int j = 0; j < quadcopterCount; j++) {
			std::cout << positions[j].toString();
			
			if (error[j]) {
				std::cout << " (with error)";
			}
			
			std::cout << std::endl;
		}
		
		int64_t currentTargetTime = (i + 1) * targetTime + startTime;
		int64_t toSleep = currentTargetTime - getNanoseconds();
		
		if (toSleep < 0) {
			std::cout << "Cannot sleep, calculation of movement took " << -toSleep << " ns to long!" << std::endl;
		} else {
			sleep(toSleep);
		}
		
		std::cout << "============================================================" << std::endl;
	}
}
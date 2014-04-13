#define DEBUG_TIME
#include "../../src/matlab/profiling.hpp"

static long int _time = 0;

/**
 * Set the current time to the given value.
 *
 * @param time The time to set.
 */
void setNanoTime(long int time)
{
	_time = time;
}

/**
 * Returns the time that was set by setNanoTime, or 0 if setNanoTime was never
 * called.
 *
 * @return The time set by setNanoTime, or 0 if setNanoTime was never called.
 */
long int getNanoTime()
{
	return _time;
}
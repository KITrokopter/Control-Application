#define DEBUG_TIME
#include "../../src/matlab/profiling.hpp"

static long int _time = 0;

void setNanoTime(long int time)
{
	_time = time;
}

long int getNanoTime()
{
	return _time;
}
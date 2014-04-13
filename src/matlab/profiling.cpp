#include "profiling.hpp"

long int getNanoTime()
{
	timespec ts;
	clock_gettime(CLOCK_REALTIME, &ts);
	return 1000000000 * ts.tv_sec + ts.tv_nsec;
}


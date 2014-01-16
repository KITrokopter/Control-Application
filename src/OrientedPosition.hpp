#ifndef ORIENTED_POSITION_HPP
#define ORIENTED_POSITION_HPP
#include <time.h>


class OrientedPosition {
public:
	//oder pointer auf array?
	int[3] getPosition();
	short[3] getOrientation();
	time_t getTimestamp();

private:
	int x;
	int y;
	int z;
	short xOrientation;
	short yOrientation;
	short zOrientation;
	time_t timestap;
};



#endif // ORIENTED_POSITION_HPP

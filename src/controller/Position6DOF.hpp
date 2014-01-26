#ifndef POSITION6DOF_HPP
#define POSITION6DOF_HPP
#include <time.h>


class Position6DOF {
public:
	//both arrays of the return pointer have size three
	int* getPosition();
	short int* getOrientation();
	int position[3];
	short int orientation[3];
	time_t getTimestamp();
	Position6DOF(int x, int y, int z, short int xOrientation, short int yOrientation, short int zOrientation);

private:
	//int position[3];
	//short int orientation[3];
	time_t timestamp;
};



#endif // POSITION6DOF_HPP

#ifndef FORMATION_HPP
#define FORMATION_HPP
#include "Position6DOF.hpp"
#include "ros/ros.h"
#include <vector>

class Formation {
public:
	Formation(int distance, int amount, Position6DOF *position);
	Formation();
	void setDistance(int distance);
	void setAmount(int amount);
	void setFormationPosition(Position6DOF *position);
	int getDistance();
	int getAmount();
	Position6DOF* getFormationPosition();

private:
	int distance;
	int amount;
	Position6DOF position[10];
};



#endif // FORMATION_HPP

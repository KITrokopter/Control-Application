#ifndef FORMATION_HPP
#define FORMATION_HPP
#include "Position6DOF.hpp"

class Formation {
public:
	Formation(int distance, int amount, Position6DOF * position);
	Formation(){};
	void setDistance(int distance);
	void setAmount(int amount);
	void setPosition(Position6DOF * position);
	int getDistance();
	int getAmount();
	Position6DOF* getPosition();
private:
	int distance;
	int amount;
	Position6DOF* position;
};



#endif // FORMATION_HPP

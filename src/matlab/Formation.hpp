#ifndef FORMATION_HPP
#define FORMATION_HPP

class Formation {
public:
	setDistance(int distance);
	setAmount(int amount);
	setPosition(int position[amount][3]);
	int getDistance();
	int setAmount();
	int[amount][3] setPosition();
private:
	int distance;
	int amount;
	int position[amount][3];
};



#endif // FORMATION_HPP

#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP
#include "OrientedPosition.hpp"
#include "Formation.hpp"

class Controller {
public:
	Controller(OrientedPosition targetPosition[], OrientedPosition currentPosition[], Formation formation);
	void initialize();
	void makeMovement();
	setTargetPosition(OrientedPosition targetPosition[]);
	buildFormation(Formation formation);
	void shutdown();
	void checkInputMovement();
	OrientedPosition[] getTargetPosition();

private:
	OrientedPostion targetPosition[], currentPosition[];
	//Brauchen wir Formation hier ueberhaupt. Wo ist die gewaehlte Formation gespeichert? Wo sind die Qudcopter-Objekte gespeichert? Wo die Kameras? Gamepad?
	Formation formation;
	int amount;
};



#endif // CONTROLLER_HPP

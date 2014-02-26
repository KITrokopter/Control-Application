#include "helpers/MovementGenerator.hpp"
#include "../src/controller/Controller.hpp"

#include <iostream>

int main(int argc, char** argv) {
	std::cout << "Start profile_control" << std::endl;

	/*
	 * TODO: needs to be initialized
	 * Controller instead of DummyPositionReceiver
	 */
	Controller *receiver = new Controller(); 
	
	std::vector<Vector> from;
	std::vector<Vector> to;
	
	from.push_back(Vector(0, 0, 0));
	to.push_back(Vector(1, 1, 1));
	from.push_back(Vector(0, 0, 1));
	to.push_back(Vector(1, 1, 2));
	from.push_back(Vector(0, 1, 0));
	to.push_back(Vector(1, 2, 1));
	from.push_back(Vector(1, 0, 0));
	to.push_back(Vector(2, 1, 1));
	from.push_back(Vector(1, 1, 1));
	to.push_back(Vector(2, 2, 2));
	
	// See MovementGenerator.hpp
	MovementGenerator generator(receiver, from, to, 0.05, 0.1, 0.01, 150, 30);
	generator.run();
}
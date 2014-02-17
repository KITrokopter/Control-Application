#ifndef IORIENTATION_CALCULATOR
#define IORIENTATION_CALCULATOR

#include <vector>
#include "../matlab/Vector.h"

class {
public:
	/**
	 * Method: getOrientation
	 * 
	 * Calculates the orientation from the given positions of the quadcopters.
	 * Classes implementing this should keep track of the positions and movement directions internally.
	 * 
	 * Parameters:
	 *   ids - The ids of the quadcopters.
	 *   positions - A new set of positions of the quadcopters.
	 *   targetMovement - The 'should' movement of the quadcopters.
	 * 
	 * Returns:
	 *   The orientations of the given quadcopters.
	 * 
	 * Values in the same position in a std::vector belong together. That means, positions.at(i) is the position of the
	 * quadcopter with id ids.at(i) that has the target movement vector targetMovement.at(i). In the return value, the
	 * orientation saved at position i is the orientation of the quadcopter with id ids.at(i).
	 */	
	virtual std::vector<Vector> getOrientation(std::vector<int> ids, std::vector<Vector> positions, std::vector<Vector> targetMovement) = 0;
};

#endif // IORIENTATION_CALCULATOR
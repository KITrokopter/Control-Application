#ifndef DUMMY_POSITION_RECEIVER_HPP
#define DUMMY_POSITION_RECEIVER_HPP

#include "IPositionReceiver.hpp"

class DummyPositionReceiver : public IPositionReceiver
{
public:
	void updatePositions(std::vector<Vector> positions, std::vector<int> ids, std::vector<int> updates);
};

#endif // DUMMY_POSITION_RECEIVER_HPP
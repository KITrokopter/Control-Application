#include "DummyPositionReceiver.hpp"

#include <ros/console.h>

void DummyPositionReceiver::updatePositions(std::vector<Vector> positions, std::vector<int> ids,
                                            std::vector<int> updates)
{
	// Do nothing, this is just a dummy.
	// ROS_DEBUG("DummyPositionReceiver: Id: %d Position: %f %f %f", ids[0],
	// positions[0].getV1(), positions[0].getV2(), positions[0].getV3());
}

void DummyPositionReceiver::setTrackingArea(TrackingArea area)
{
	ROS_DEBUG("DummyPositionReceiver: tracking area set");
}
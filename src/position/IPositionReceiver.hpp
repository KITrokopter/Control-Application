#ifndef IPOSITION_RECEIVER
#define IPOSITION_RECEIVER

#include <vector>
#include "../matlab/Vector.h"
#include "../matlab/TrackingArea.h"

/**
 * An interface for classes that want to receive the position data from the
 * tracking system.
 *
 * @author Sebastian Schmidt
 */
class IPositionReceiver {
public:
	/**
	 * Takes the positions of all quadcopters that were updated.
	 * Automatically generates new control information for the quadcopters
	 * and sends it via ROS.
	 *
	 * @param positions The positions of the quadcopters.
	 * @param ids The ids of the quadcopters.
	 * @param updates The amount of image data that went into the position of
	 * the quadcopter since the last call of this method.
	 *             Can be ignored for now, but may be useful later.
	 *
	 * Values in the same position in a std::vector belong together. That means,
	 * positions[i] is the position of the
	 * quadcopter with id ids[i] that was generated with updates[i] images since
	 * the last call of this method.
	 */
	virtual void updatePositions(std::vector<Vector> positions, std::vector<int> ids, std::vector<int> updates) = 0;

	/**
	 * Sets the tracking area to use for controlling the quadcopters.
	 *
	 * @param area The tracking area.
	 */
	virtual void setTrackingArea(TrackingArea area) = 0;
};

#endif // IPOSITION_RECEIVER
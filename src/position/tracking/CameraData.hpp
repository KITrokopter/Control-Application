#pragma once

#include "../../matlab/Vector.h"

/**
 * Container for camera data.
 * This contains data for exactly one quadcopter
 * extracted from exactly one image.
 *
 * @author Sebastian Schmidt
 */
class CameraData {
public:
	/**
	 * The vector pointing from the camera to the quadcopter,
	 * in the cameras own coordinate system.
	 */
	Vector cameraVector;

	/** The number of the camera. */
	int camNo;

	/** The id of the quadcopter. */
	int quadcopterId;

	/**
	 * The time the image that this data was extracted from
	 * was taken.
	 */
	long int time;

	/** True if the camera data is valid. */
	bool valid;
};
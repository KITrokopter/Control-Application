#ifndef POSITION_H
#define POSITION_H
#include "engine.h"
#include "AmccCalibration.h"
#include "../position/ChessboardData.hpp"
#include "Vector.h"
#include "Line.h"
#include "Matlab.h"
#include "AmccCalibration.h"
#include "engine.h"
#include <vector>

class Position {
private: 
	// enigne pointer to matlab application
	Engine *ep;
	// number of cameras
	int numberCameras;
	// saves whether the cameras are multicalibrated or not;
	bool transformed;
	// saves whether the position and translation is calculated of all cameras;
	bool calibrated;
	/* 
	 * (quadPos[i])[j] is the position of quadrocopter with ID i and camera with ID j
	 *  default value is nan, maximum amount of quadcopter is 50, of cameras is 20
	 */
	std::vector< std::vector <Vector> > quadPos;
	/* 
	 * oldPos[i] is the last calculated position of quadcopter with ID i, default value is nan
	 * maximum amount of quadcopters is 50
	 */ 
	std::vector<Vector> oldPos;

	/* 
		Variables saved in matlab:

		cameraPosition_cameraId: real position of camera with cameraId
		cameraOrientation_cameraId: real orientation of camera with cameraId

		rotationMatrix: rotationmatrix to calculate camera system of camera 0 to real coordinate system

		camera values in camera coordinate system with cameraId
		rotMatCamCoord_cameraId: rodrigues(omc_left_x)
		transVectCamCoord_cameraId

		camera values in real coordinationsystem
		rotationMatrixCamera_cameraId: rotationmatrix of camera with cameraId, is rodrigues(omc_left_x)
		translationVectorCamera_cameraId: translation vector of camera with cameraId, is Tc_left_x
		(x is first not supressed image)
	*/

public:
	// maximal amount of quadcopters is 50, maximal amount of cameras is 20
    	Position();
    	Position(Engine *ep, int numberCameras);

	// calibrating with amcc toolbox, saving matlab variables in workspace after calibration
        bool calibrate(ChessboardData *chessboardData, int numberCameras);
	// calculating angle between vector u and vector v
	double getAngle(Vector u, Vector v);
	// setter
	void setNumberCameras(int numberCameras);

	// transforming coordinate system with positiv or negative angle (sign should only be -1 or 1)	
	void angleTry(int sign);	
	// transforming coordinate system of camera 0 to coordinate system where all cameras are on the xy-plane 
	Vector getCoordinationTransformation(Vector w, int cameraId);

	/// quad is vector of camera with cameraId, that points to quadcopter with quadcopterId, returns (Nan, NaN, NaN) the first time, the position is calculated, or if not all cameras did track it yet	
    	Vector updatePosition(Vector quad, int cameraId, double quadcopterId);
	/// should only be called once, loads the output of the position of a camera in coordinate system of camera 0
        Vector getPositionInCameraCoordination(int cameraId);
	/// should only be called once, loads the output of the orientation of a camera in coordinate system of camera 0   
	Vector getOrientationInCameraCoordination(int cameraId);
	/// should only be called once, value is saved in variable cameraPosition_cameraId, is in real coordinate system
	Vector getPosition(int cameraId);
	/// should only be called once, value is saved in variable cameraOrientation_cameraId, is in real coordinate system
	Vector getOrientation(int cameraId);
	
	// loads values of amcc toolbox calibration of camera with cameraId in matlab workspace
    	void loadValues(int cameraId);
};

#endif // POSITION_H //

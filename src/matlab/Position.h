#ifndef POSITION_H
#define POSITION_H
#include "engine.h"
#include "AmccCalibration.h"
#include "../position/ChessboardData.hpp"
#include "Vector.h"
#include "Line.h"
#include "Matrix.h"
#include "Matlab.h"
#include "AmccCalibration.h"
#include "engine.h"
#include <vector>
#include "TrackingArea.h"

class Position {
private: 
	// enigne pointer to matlab application
	Engine *ep;
	// number of cameras
	int numberCameras;
	// saves whether the cameras are multicalibrated or not;
	bool transformed;

    // distance of last seen position and actual position
    double distance;

	// TrackingArea of cameras
    TrackingArea tracking;

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
	

	// output of amcc toolbox
	// camera position in camera coordinate system of camera 0
	std::vector<Vector> camCoordCameraPos;
	// camera orientation in camera coordinate system of camera 0
	std::vector<Vector> camCoordCameraOrient;
	// camera rotation matrices of cameras to rotate in coordinate system of camera 0
	std::vector<Matrix> camRotMat;


	// calculated of results of amcc toolbox
	// real position of cameras
	std::vector<Vector> realCameraPos;
	// real orientation of cameras
	std::vector<Vector> realCameraOrient;
	// rotationmatrix to calculate coordinate system of camera 0 to real coordinate system	
	Matrix rotationMatrix;

	/// calculates camera position, that is saved in realCameraPos[cameraId]
	void calculatePosition(int cameraId);
	/// calculates orientation, that is saved in realCameraOrient[cameraId]
	void calculateOrientation(int cameraId);

	// transforming coordinate system with positiv or negative angle (sign should only be -1 or 1)	
	void angleTry(int sign);
	// loads values of amcc toolbox calibration of camera with cameraId in matlab workspace
	void loadValues(int cameraId);

	std::vector<int> imageAge;
public:
	// maximal amount of quadcopters is 50, maximal amount of cameras is 20
	Position();
	Position(Engine *ep, int numberCameras);

	// checks whether the calibration has been made successfully
	bool calibratedYet(int numberCameras);

	// calibrating with amcc toolbox, saving matlab variables in workspace after calibration
	bool calibrate(ChessboardData *chessboardData, int numberCameras);
	// calculating angle between vector u and vector v
	double getAngle(Vector u, Vector v);
	// setter
	void setNumberCameras(int numberCameras);

	// sets the tracking area after calibration
	void setTrackingArea(double maxRange);
	TrackingArea getTrackingArea();	

	// transforming coordinate system of camera 0 to coordinate system where all cameras are on the xy-plane, returns vector w in real co-system
	Vector calculateCoordinateTransformation(Vector w, int cameraId);

    // quad is vector of camera with cameraId, that points to quadcopter with quadcopterId, returns (Nan, NaN, NaN) the first time, the position is calculated, or if not all cameras did track it yet
	Vector updatePosition(Vector quad, int cameraId, int quadcopterId);

	
	// returns position of camera with cameraId, returns NAN, if not yet calibrated
	Vector getPosition(int cameraId);

    // returns distance between last seen position and last calculated position
    double getDistance();

    // saves distorion coefficient of camera with cameraId in distCoeff
    void getDistortionCoefficients(int cameraId, double* distCoeff);

    // calculates intrinsic matrix as can be seen here: http://www.vision.caltech.edu/bouguetj/calib_doc/htmls/parameters.html
    Matrix getIntrinsicsMatrix(int cameraId);

};

#endif // POSITION_H //

#ifndef POSITION_CALCULATOR_H
#define POSITION_CALCULATOR_H
#include "engine.h"
#include "AmccCalibration.h"
#include "../position/ChessboardData.hpp"
#include "Vector.h"
#include "Line.h"
#include "Matrix.h"
#include "TrackingMath.h"
#include "AmccCalibration.h"
#include <engine.h>
#include <vector>
#include "TrackingArea.h"
#include "../position/tracking/CameraData.hpp"

#include <opencv2/core/core.hpp>

class PositionCalculator {
private:

    /**
     * ep enigne pointer to matlab application
     */
    Engine *ep;

    /**
     * numberCameras is the number of cameras
     */
    int numberCameras;

    /**
     * transformed: saves whether the cameras are multicalibrated or not
     */
    bool transformed;

    /**
     * interpolationDependent: saves whether interpolationFactor should always be 0.5 (false) or should be dependent to the distance (true)
     */
    bool interpolationDependent;

    /**
     * distance of last seen position and actual position
     */
    double distance;

    /**
     * average distance of new interpolated point to perpendicular foot points
     */
    double error;

    /**
     * TrackingArea of cameras
     */
    TrackingArea tracking;

    /**
	 * (quadPos[i])[j] is the position of quadrocopter with ID i and camera with ID j
	 *  default value is nan, maximum amount of quadcopter is 50, of cameras is 20
	 */
	std::vector< std::vector <Vector> > quadPos;

    /**
     * oldPos[i] is the last calculated position of quadcopter with ID i, default value is nan
     * maximum amount of quadcopters is 50
     */
    std::vector<Vector> oldPos;
	

    /// output of amcc toolbox

    /**
     * camCoordCameraPos: camera position in camera coordinate system of camera 0
     */
    std::vector<Vector> camCoordCameraPos;

    /**
     * camCoordCameraOrient: camera orientation in camera coordinate system of camera 0
     */
    std::vector<Vector> camCoordCameraOrient;

    /**
     * @brief camRotMat: camera rotation matrices of cameras to rotate in coordinate system of camera 0
     */
    std::vector<Matrix> camRotMat;



    /// calculated of results of amcc toolbox

    /**
     * realCameraPos: real position of cameras
     */
    std::vector<Vector> realCameraPos;

    /**
     * @brief realCameraOrient: real orientation of cameras
     */
    std::vector<Vector> realCameraOrient;

    /**
     * rotationMatrix to calculate coordinate system of camera 0 to real coordinate system
     */
    Matrix rotationMatrix;

    /**
     * calculates camera position, that is saved in realCameraPos[cameraId]
     * @param cameraId of camera
     */
    void calculatePosition(int cameraId);

    /**
     * calculates orientation, that is saved in realCameraOrient[cameraId]
     * @param cameraId of camera
     */
    void calculateOrientation(int cameraId);

    /**
     * transforming coordinate system with positiv or negative angle
     * @param sign sign of angle
     */
    void angleTry(int sign);

    /**
     * loads values of amcc toolbox stereo calibration of camera with cameraId in matlab workspace
     * @param cameraId of camera
     */
    void loadValues(int cameraId);

    /**
     * initialize start values for constructor
     */
    void initialize();

    /**
     * imageAge[i]: number of new received images that hasn't been sent of camera with cameraId i
     */
	std::vector<int> imageAge;

    /**
     * calculates tracking area and sets it after calibration
     * @param maxRange
     */
    void setTrackingArea(double maxRange);

    /**
     * m is Matlab instance to calculate perpendicular points
     */
    TrackingMath m;

public:

    /**
     * constructor, maximal amount of quadcopters is 50, maximal amount of cameras is 20.
     */
    PositionCalculator();

    /**
     * constructor, maximal amount of quadcopters is 50, maximal amount of cameras is 20.
     * @param interpolationDependent is interpolation factor that is always the same if false
     */
    PositionCalculator(bool interpolationDependent);

    /**
     * constructor, maximal amount of quadcopters is 50.
     * @param ep Engine pointer
     * @param numberCameras number of cameras
     */
    PositionCalculator(Engine *ep, int numberCameras);

    /**
     * constructor, maximal amount of quadcopters is 50.
     * @param ep Engine pointer
     * @param numberCameras number of cameras
     * @param interpolationDependent is interpolation factor that is always the same if false
     */
    PositionCalculator(Engine *ep, int numberCameras, bool interpolationDependent);

    /**
      * destroys matlab engine pointer.
      */
    ~PositionCalculator();

    /**
     * checks whether the calibration has been made successfully
     * @param numberCameras number of cameras
     * @return true, if calibration has been made successfully, false otherwise
     */
    bool calibratedYet(int numberCameras);

    /**
     * calibrating with amcc toolbox, saving matlab variables in workspace after calibration.
     * @param chessboardData of checkboard
     * @param numberCameras number of cameras
     * @return true if everything worked
     */
    bool calibrate(ChessboardData *chessboardData, int numberCameras);

    /**
     * setter.
     * @param numberCameras number of cameras
     */
	void setNumberCameras(int numberCameras);

    /**
     * getter.
     * @return tracking area of calibrated area.
     */
	TrackingArea getTrackingArea();	

    /**
     * transforming coordinate system of camera 0 to coordinate system where all cameras are on the xy-plane.
     * @param w vector that should be rotated
     * @return vector w in real co-system.
     */
    Vector calculateCoordinateTransformation(Vector w);

    /**
     * updates new position of quadcopter.
     * @param cameraLines are lines of some cameras to the same tracked quadcopter
     * @return new position of quadcopter
     */
    Vector updatePosition(std::vector<CameraData> cameraLines);

	
    /**
     * getter.
     * @param cameraId id of camera
     * @return position of camera with cameraId, NAN if ID is invalid
     */
    Vector getPosition(int cameraId);

    /**
     * getter.
     * @return distance between last seen position and last calculated position. First time is 0.
     */
    double getDistance();

    /**
     * loads distortion coefficients of camera with cameraId based on amcc toolbox results.
     * @param cameraId Id of camera
     * @return distortion coefficients of camera with cameraId
     */
    cv::Mat getDistortionCoefficients(int cameraId);

    /**
     * loads and calculates intrinsic matrix of camera with cameraId based on amcc toolbox results.
     * @param cameraId Id of camera
     * @return intrinsic matrix calculated as described here: http://www.vision.caltech.edu/bouguetj/calib_doc/htmls/parameters.html
     */
    cv::Mat getIntrinsicsMatrix(int cameraId);

    /**
     * getter.
     * @param cameraId
     * @return rotation matrix , that transforms camera co-system of camera with cameraId in real co-system
     */
    Matrix getRotationMatrix(int cameraId);

    /**
     * getter.
     * @return average distance of perpendicular foot points to new interpolated point
     */
    double getError();

};

#endif // POSITION_CALCULATOR_H //

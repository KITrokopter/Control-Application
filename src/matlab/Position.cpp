#include "Position.h"
#include "Calibration.h"

Position::Position(Engine *ep)
{
    this->ep = ep;
    calib = *(new Calibration());
}

void Position::calibrate(ChessboardData *chessboardData, int cameraId) {
	// Let dominik calculate the angles.
	// If first call, this is camera A, if second call, this is camera B,
	// if third call, this is camera C, if fourth call, error (do whatever you want, or ignore).
	// Calculate the rotation matrix and translation vector, given the angles and the position of the camera (A, B or C).
	
    // if B
    //calib.setChangeOfBasisVectorB(cameraData.getAlpha(), cameraData.getGamma());
    //calib.setTranslationVectorB(cameraData.getSigma(), cameraData.getHa());
    // if C
    //calib.setChangeOfBasisVectorB(cameraData.getAlpha(), cameraData.getGamma());
    //calib.setTranslationVectorB(cameraData.getSigma(), cameraData.getHa());
}

Vector Position::updatePosition(Vector v, int cameraId, double quadcopterId) {

}

Vector getPosition(int cameraId) {
    //return calib.getTranslationVectorx();
}

Vector getOrientation(int cameraId) {

}

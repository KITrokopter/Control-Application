#include "Position.h"
#include "Calibration.h"

Position::Position(Engine *ep)
{
    this->ep = ep;
    calib = *(new Calibration());
}

void Position::calibrate(CameraData cameraData, double cameraId) {
    // if B
    calib.setChangeOfBasisVectorB(cameraData.getAlpha(), cameraData.getGamma());
    calib.setTranslationVectorB(cameraData.getSigma(), cameraData.getHa());
    // if C
    calib.setChangeOfBasisVectorB(cameraData.getAlpha(), cameraData.getGamma());
    calib.setTranslationVectorB(cameraData.getSigma(), cameraData.getHa());
}

Vector Position::updatePosition(Vector v, double cameraId, double quadcopterId) {

}

Vector getPosition(double cameraId) {
    //return calib.getTranslationVectorx();
}

Vector getOrientation(double cameraId) {

}

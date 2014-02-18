#include "Position.h"
#include "Calibration.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <iostream>

Position::Position()
{
    Engine *ep;
    // starts a MATLAB process
    if (!(ep = engOpen(""))) {
            fprintf(stderr, "\nCan't start MATLAB engine\n");
    } else {
        this->ep = ep;
    }
    calib = *(new Calibration());
}

Position::Position(Engine *ep)
{
    this->ep = ep;
    calib = *(new Calibration());
}

void Position::calibrate(CameraData cameraData, int cameraId) {
    // if B
    calib.setChangeOfBasisVectorB(cameraData.getAlpha(), cameraData.getGamma());
    calib.setTranslationVectorB(cameraData.getSigma(), cameraData.getHa());
    // if C
    calib.setChangeOfBasisVectorB(cameraData.getAlpha(), cameraData.getGamma());
    calib.setTranslationVectorB(cameraData.getSigma(), cameraData.getHa());
}

Vector Position::updatePosition(Vector v, int cameraId, double quadcopterId) {

}

Vector getPosition(int cameraId) {
    //return calib.getTranslationVectorx();
}

Vector getOrientation(int cameraId) {

}

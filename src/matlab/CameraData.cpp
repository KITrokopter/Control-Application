#include "CameraData.h"

CameraData::CameraData()
{
    alpha = 0;
    gamma = 0;
    sigma = 0;
    ha = 0;
}

double CameraData::getAlpha() {
    return alpha;
}

double CameraData::getGamma() {
    return gamma;
}

double CameraData::getSigma() {
    return sigma;
}

double CameraData::getHa() {
    return ha;
}


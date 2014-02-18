#ifndef CAMERADATA_H
#define CAMERADATA_H

class CameraData {
private: double alpha, gamma, sigma, ha;
public:
    CameraData();
    double getAlpha();
    double getGamma();
    double getSigma();
    double getHa();
};

#endif // CAMERADATA_H

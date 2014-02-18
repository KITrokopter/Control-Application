#ifndef CALIBRATION_H
#define CALIBRATION_H
#include "engine.h"
#include "Vector.h"

class Calibration {
private: Engine *ep; 
	 Vector translationA, translationB, translationC;
public:
    Calibration();
    Calibration(Engine *ep);
    void setChangeOfBasisVectorB(double alpha, double gamma);
    void setChangeOfBasisVectorC(double alpha, double gamma);
    void setTranslationVectorB(double sigma, double ha);
    void setTranslationVectorC(double sigma, double ha);
    Vector getTranslationVectorB();
    Vector getTranslationVectorC();
};

#endif // CALIBRATION_H

#include "Calibration.h"
#include "engine.h"
#include "Vector.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <iostream>

Calibration::Calibration()
{
    Engine *ep;
    // starts a MATLAB process
    if (!(ep = engOpen(""))) {
        fprintf(stderr, "\nCan't start MATLAB engine\n");
    } else {
        this->ep = ep;
    }
    translationA = *(new Vector());
    translationB = *(new Vector());
    translationC = *(new Vector());
}

Calibration::Calibration(Engine *ep) {
    this->ep = ep;
    translationA = *(new Vector());
    translationB = *(new Vector());
    translationC = *(new Vector());
}

/*
 * alpha: rotation xy-plain, gamma: rotation xz-plain.
 */
void Calibration::setChangeOfBasisVectorB(double alpha, double gamma) {
    //input in matlab
    double d[1] = {alpha};
    double e[1] = {gamma};
    mxArray *a = mxCreateDoubleMatrix(1, 1, mxREAL);
    memcpy((void*)mxGetPr(a), (void*)d, sizeof(d));
    engPutVariable(ep, "a", a);
    mxArray *g = mxCreateDoubleMatrix(1, 1, mxREAL);
    memcpy((void*)mxGetPr(g), (void*)e, sizeof(e));
    engPutVariable(ep, "g", g);

    // Rotationmatrices
    engEvalString(ep, "Rz = [cos(a) -sin(a) 0; sin(a) cos(a) 0; 0 0 1];");
    engEvalString(ep, "Ry = [cos(g) 0 sin(g); 0 1 0; -sin(g) 0 cos(g)];");
    engEvalString(ep, "M = Rz*Ry");
    engEvalString(ep, "Mc = [cos(120) -sin(120) 0; sin(120) cos(120) 0; 0 0 1] * M");
}

void Calibration::setChangeOfBasisVectorC(double alpha, double gamma) {
    //input in matlab
    double d[1] = {alpha};
    double e[1] = {gamma};
    mxArray *a = mxCreateDoubleMatrix(1, 1, mxREAL);
    memcpy((void*)mxGetPr(a), (void*)d, sizeof(d));
    engPutVariable(ep, "a", a);
    mxArray *g = mxCreateDoubleMatrix(1, 1, mxREAL);
    memcpy((void*)mxGetPr(g), (void*)e, sizeof(e));
    engPutVariable(ep, "g", g);

    // Rotationmatrices
    engEvalString(ep, "Rz = [cos(a) -sin(a) 0; sin(a) cos(a) 0; 0 0 1];");
    engEvalString(ep, "Ry = [cos(g) 0 sin(g); 0 1 0; -sin(g) 0 cos(g)];");
    engEvalString(ep, "M = Rz*Ry");
    engEvalString(ep, "Mb = [cos(-120) -sin(-120) 0; sin(-120) cos(-120) 0; 0 0 1] * M");
}

/*
 * sigma is the vertical length of the camera range
 * ha, is the distance between the plain of the chessboard and the camera.
 * calculates the translationVector of B
 */
void Calibration::setTranslationVectorB(double sigma, double ha) {
    translationB = *(new Vector(ha, sigma/2, 0));
}

/*
 * sigma is the vertical length of the camera range
 * ha, is the distance between the plain of the chessboard and the camera.
 * calculates the translationVector of C
 */
void Calibration::setTranslationVectorC(double sigma, double ha) {
    translationC = *(new Vector(ha, -sigma/2, 0));
}

Vector Calibration::getTranslationVectorB() {
    return translationB;
}

Vector Calibration::getTranslationVectorC() {
    return translationC;
}

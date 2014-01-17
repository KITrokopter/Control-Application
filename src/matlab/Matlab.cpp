/*
 * Matlab.cpp
 *
 *  Created on: 13.01.2014
 *      Author: daniela
 */

#include "Matlab.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <iostream>
#include "engine.h"
#define  BUFSIZE 256



int main()
{
    double a = 1;           // vector1 = [a, b, c]
    double b = 0;
    double c = 2;
    double d = 1;           // vector2 = [d, e, f], gerade = vector1+r*(vector2-vector1)
    double e = 2;
    double f = 3;
    double g = 2;           // vector3 = [g, h, i] Punkt
    double h = 3;
    double i = 1;
    Engine *ep;
    mxArray *p11, *p12, *p13, *p21, *p22, *p23, *p31, *p32, *p33 = NULL;
    mxArray *result;
    char buffer[BUFSIZE+1];
    double dataa[1] = {a};
    double datab[1] = {b};
    double datac[1] = {c};
    double datad[1] = {d};
    double datae[1] = {e};
    double dataf[1] = {f};
    double datag[1] = {g};
    double datah[1] = {h};
    double datai[1] = {i};
/*
 * Call engOpen with a NULL string. This starts a MATLAB process
 * on the current host using the command "matlab".
 */
    if (!(ep = engOpen(""))) {
        fprintf(stderr, "\nCan't start MATLAB engine\n");
        return EXIT_FAILURE;
    }
// Abstand Punkt P3 von Vektor / Gerade

    /*
     * Create variables for our data
     */
    p11 = mxCreateDoubleMatrix(1, 1, mxREAL);
    memcpy((void *)mxGetPr(p11), (void *)dataa, sizeof(dataa));
    p12 = mxCreateDoubleMatrix(1, 1, mxREAL);
    memcpy((void *)mxGetPr(p12), (void *)datab, sizeof(datab));
    p13 = mxCreateDoubleMatrix(1, 1, mxREAL);
    memcpy((void *)mxGetPr(p13), (void *)datac, sizeof(datac));
    p21 = mxCreateDoubleMatrix(1, 1, mxREAL);
    memcpy((void *)mxGetPr(p21), (void *)datad, sizeof(datad));
    p22 = mxCreateDoubleMatrix(1, 1, mxREAL);
    memcpy((void *)mxGetPr(p22), (void *)datae, sizeof(datae));
    p23 = mxCreateDoubleMatrix(1, 1, mxREAL);
    memcpy((void *)mxGetPr(p23), (void *)dataf, sizeof(dataf));
    p31 = mxCreateDoubleMatrix(1, 1, mxREAL);
    memcpy((void *)mxGetPr(p31), (void *)datag, sizeof(datag));
    p32 = mxCreateDoubleMatrix(1, 1, mxREAL);
    memcpy((void *)mxGetPr(p32), (void *)datah, sizeof(datah));
    p33 = mxCreateDoubleMatrix(1, 1, mxREAL);
    memcpy((void *)mxGetPr(p33), (void *)datai, sizeof(datai));

    /*
     * Place the variables into the MATLAB workspace
     */

    engPutVariable(ep, "p11", p11);

    engPutVariable(ep, "p12", p12);
    engPutVariable(ep, "p13", p13);
    engEvalString(ep, "vector1 = [p11,p12,p13];");
    engPutVariable(ep, "p21", p21);
    engPutVariable(ep, "p22", p22);
    engPutVariable(ep, "p23", p23);
    engEvalString(ep, "vector2 = [p21,p22,p23];");
    engPutVariable(ep, "p31", p31);
    engPutVariable(ep, "p32", p32);
    engPutVariable(ep, "p33", p33);
    engEvalString(ep, "vector3 = [p31,p32,p33];");
    engEvalString(ep, "directionVector = [vector2 - vector1];"); // line: v1+r*directionVector

    result = engGetVariable(ep, "directionVector");
    printf("directionVector is [%f, %f, %f]\n", mxGetPr(result)[0], mxGetPr(result)[1], mxGetPr(result)[2]);

    engEvalString(ep, "d = dot(vector3, directionVector);");

    result = engGetVariable(ep, "d");
    printf("scalarproduct is %f\n", mxGetPr(result)[0]);

    engEvalString(ep, "syms r;");
    engEvalString(ep, "vector4 = vector1 + r*directionVector;");
    engEvalString(ep, "product = dot(vector3,vector4)");
    engEvalString(ep, "r = solve(product == d, r)");
    engEvalString(ep, "x = cast(r, 'double');");

    result = engGetVariable(ep, "x");
    printf("r is %f\n", mxGetPr(result)[0]);

    engEvalString(ep, "result = vector1 + x*directionVector");

    result = engGetVariable(ep, "result");
    printf("result is [%f, %f, %f]\n", mxGetPr(result)[0], mxGetPr(result)[1], mxGetPr(result)[2]);

    mxDestroyArray(p11);
    mxDestroyArray(p12);
    mxDestroyArray(p13);
    mxDestroyArray(p21);
    mxDestroyArray(p22);
    mxDestroyArray(p23);
    mxDestroyArray(p31);
    mxDestroyArray(p32);
    mxDestroyArray(p33);
    mxDestroyArray(result);
    engClose;
    return EXIT_SUCCESS;
}

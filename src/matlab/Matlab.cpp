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

struct vector {
    double v1;
    double v2;
    double v3;
};

void enterVariablesOneLine(struct vector a, struct vector u, struct vector b, Engine *ep) {
     mxArray *a1, *a2, *a3, *b1, *b2, *b3, *u1, *u2, *u3;

     double dataa1[1] = {a.v1};
     double dataa2[1] = {a.v2};
     double dataa3[1] = {a.v3};

     double datau1[1] = {u.v1};
     double datau2[1] = {u.v2};
     double datau3[1] = {u.v3};

     double datab1[1] = {b.v1};
     double datab2[1] = {b.v2};
     double datab3[1] = {b.v3};

     a1 = mxCreateDoubleMatrix(1, 1, mxREAL);
     memcpy((void *)mxGetPr(a1), (void *)dataa1, sizeof(dataa1));
     engPutVariable(ep, "a1", a1);
     a2 = mxCreateDoubleMatrix(1, 1, mxREAL);
     memcpy((void *)mxGetPr(a2), (void *)dataa2, sizeof(dataa2));
     engPutVariable(ep, "a2", a2);
     a3 = mxCreateDoubleMatrix(1, 1, mxREAL);
     memcpy((void *)mxGetPr(a3), (void *)dataa3, sizeof(dataa3));
     engPutVariable(ep, "a3", a3);

     u1 = mxCreateDoubleMatrix(1, 1, mxREAL);
     memcpy((void *)mxGetPr(u1), (void *)datau1, sizeof(datau1));
     engPutVariable(ep, "u1", u1);
     u2 = mxCreateDoubleMatrix(1, 1, mxREAL);
     memcpy((void *)mxGetPr(u2), (void *)datau2, sizeof(datau2));
     engPutVariable(ep, "u2", u2);
     u3 = mxCreateDoubleMatrix(1, 1, mxREAL);
     memcpy((void *)mxGetPr(u3), (void *)datau3, sizeof(datau3));
     engPutVariable(ep, "u3", u3);

     b1 = mxCreateDoubleMatrix(1, 1, mxREAL);
     memcpy((void *)mxGetPr(b1), (void *)datab1, sizeof(datab1));
     engPutVariable(ep, "b1", b1);
     b2 = mxCreateDoubleMatrix(1, 1, mxREAL);
     memcpy((void *)mxGetPr(b2), (void *)datab2, sizeof(datab2));
     engPutVariable(ep, "b2", b2);
     b3 = mxCreateDoubleMatrix(1, 1, mxREAL);
     memcpy((void *)mxGetPr(b3), (void *)datab3, sizeof(datab3));
     engPutVariable(ep, "b3", b3);

     mxDestroyArray(a1);
     mxDestroyArray(a2);
     mxDestroyArray(a3);
     mxDestroyArray(u1);
     mxDestroyArray(u2);
     mxDestroyArray(u3);
     mxDestroyArray(b1);
     mxDestroyArray(b2);
     mxDestroyArray(b3);

}

void enterVariablesTwoLines(struct vector a, struct vector u, struct vector b, struct vector v, Engine *ep) {
     mxArray *a1, *a2, *a3, *b1, *b2, *b3, *u1, *u2, *u3, *v1, *v2, *v3;

     double dataa1[1] = {a.v1};
     double dataa2[1] = {a.v2};
     double dataa3[1] = {a.v3};

     double datau1[1] = {u.v1};
     double datau2[1] = {u.v2};
     double datau3[1] = {u.v3};

     double datab1[1] = {b.v1};
     double datab2[1] = {b.v2};
     double datab3[1] = {b.v3};

     double datav1[1] = {v.v1};
     double datav2[1] = {v.v2};
     double datav3[1] = {v.v3};

     a1 = mxCreateDoubleMatrix(1, 1, mxREAL);
     memcpy((void *)mxGetPr(a1), (void *)dataa1, sizeof(dataa1));
     engPutVariable(ep, "a1", a1);
     a2 = mxCreateDoubleMatrix(1, 1, mxREAL);
     memcpy((void *)mxGetPr(a2), (void *)dataa2, sizeof(dataa2));
     engPutVariable(ep, "a2", a2);
     a3 = mxCreateDoubleMatrix(1, 1, mxREAL);
     memcpy((void *)mxGetPr(a3), (void *)dataa3, sizeof(dataa3));
     engPutVariable(ep, "a3", a3);

     u1 = mxCreateDoubleMatrix(1, 1, mxREAL);
     memcpy((void *)mxGetPr(u1), (void *)datau1, sizeof(datau1));
     engPutVariable(ep, "u1", u1);
     u2 = mxCreateDoubleMatrix(1, 1, mxREAL);
     memcpy((void *)mxGetPr(u2), (void *)datau2, sizeof(datau2));
     engPutVariable(ep, "u2", u2);
     u3 = mxCreateDoubleMatrix(1, 1, mxREAL);
     memcpy((void *)mxGetPr(u3), (void *)datau3, sizeof(datau3));
     engPutVariable(ep, "u3", u3);

     b1 = mxCreateDoubleMatrix(1, 1, mxREAL);
     memcpy((void *)mxGetPr(b1), (void *)datab1, sizeof(datab1));
     engPutVariable(ep, "b1", b1);
     b2 = mxCreateDoubleMatrix(1, 1, mxREAL);
     memcpy((void *)mxGetPr(b2), (void *)datab2, sizeof(datab2));
     engPutVariable(ep, "b2", b2);
     b3 = mxCreateDoubleMatrix(1, 1, mxREAL);
     memcpy((void *)mxGetPr(b3), (void *)datab3, sizeof(datab3));
     engPutVariable(ep, "b3", b3);

     v1 = mxCreateDoubleMatrix(1, 1, mxREAL);
     memcpy((void *)mxGetPr(v1), (void *)datav1, sizeof(datav1));
     engPutVariable(ep, "v1", v1);
     v2 = mxCreateDoubleMatrix(1, 1, mxREAL);
     memcpy((void *)mxGetPr(v2), (void *)datav2, sizeof(datav2));
     engPutVariable(ep, "v2", v2);
     v3 = mxCreateDoubleMatrix(1, 1, mxREAL);
     memcpy((void *)mxGetPr(v3), (void *)datav3, sizeof(datav3));
     engPutVariable(ep, "v3", v3);

     mxDestroyArray(a1);
     mxDestroyArray(a2);
     mxDestroyArray(a3);
     mxDestroyArray(u1);
     mxDestroyArray(u2);
     mxDestroyArray(u3);
     mxDestroyArray(b1);
     mxDestroyArray(b2);
     mxDestroyArray(b3);
     mxDestroyArray(v1);
     mxDestroyArray(v2);
     mxDestroyArray(v3);

}
/*
  Both Lines are defined by f = a + r*u, g = b + s*v, u and v are already direction vectors.
*/
struct vector* perpFootTwoLines(struct vector a, struct vector u, struct vector b, struct vector v, Engine *ep) {
    enterVariablesTwoLines(a, u, b, v, ep);
    mxArray *resultf, *resultg;
    engEvalString(ep, "a = [a1, a2, a3];");
    engEvalString(ep, "u = [u1, u2, u3];");
    engEvalString(ep, "b = [b1, b2, b3];");
    engEvalString(ep, "v = [v1, v2, v3];");
    engEvalString(ep, "n = cross(u, v);");
    engEvalString(ep, "A = [-u1 -n(1) v1; -u2 -n(2) v2; -u3 -n(3) v3];");
    engEvalString(ep, "dif = [a1 - b1, a2 - b2, a3 - b3]");
    engEvalString(ep, "bb = [dif(1); dif(2); dif(3)];");
    engEvalString(ep, "x = inv(A)*bb");
    engEvalString(ep, "perpf = a + x(1,1) * u");
    engEvalString(ep, "perpg = b + x(3,1) * v");
    resultf = engGetVariable(ep, "perpf");
    printf("Lotfußpunkt von f ist [%f, %f, %f]\n", mxGetPr(resultf)[0], mxGetPr(resultf)[1], mxGetPr(resultf)[2]);
    resultg = engGetVariable(ep, "perpg");
    printf("Lotfußpunkt von g ist [%f, %f, %f]\n", mxGetPr(resultg)[0], mxGetPr(resultg)[1], mxGetPr(resultg)[2]);
    struct vector perpFootf, perpFootg;
    perpFootf.v1 = mxGetPr(resultf)[0];
    perpFootf.v2 = mxGetPr(resultf)[1];
    perpFootf.v3 = mxGetPr(resultf)[2];
    perpFootf.v1 = mxGetPr(resultg)[0];
    perpFootf.v2 = mxGetPr(resultg)[1];
    perpFootf.v3 = mxGetPr(resultg)[2];
    struct vector perpFoot[2] = {perpFootf, perpFootg};
    mxDestroyArray(resultf);
    mxDestroyArray(resultg);
    return perpFoot;
}

struct vector perpFootOneLine(struct vector a, struct vector u, struct vector b, Engine *ep) {
    enterVariablesOneLine(a, u, b, ep);
    mxArray *result;
    engEvalString(ep, "a = [a1,a2,a3];");
    engEvalString(ep, "u = [u1,u2,u3];");
    engEvalString(ep, "b = [b1,b2,b3];");
    engEvalString(ep, "d = dot(b, u);");
    result = engGetVariable(ep, "d");

    /*
      nicer solution with Sym MathToolbox
        engEvalString(ep, "syms r;");
        engEvalString(ep, "vector = a + r*u;");
        engEvalString(ep, "product = dot(b,vector)");
        engEvalString(ep, "r = solve(product == d, r)");
        engEvalString(ep, "x = cast(r, 'double');");
    */

    engEvalString(ep, "x = (a1*b1+a2*b2+a3*b3-d)/(-u1*b1-u2*b2-u3*b3)");
    engEvalString(ep, "result = a + x*u");

    result = engGetVariable(ep, "result");
    printf("result is [%f, %f, %f]\n", mxGetPr(result)[0], mxGetPr(result)[1], mxGetPr(result)[2]);
    struct vector perpFoot;
    perpFoot.v1 = mxGetPr(result)[0];
    perpFoot.v2 = mxGetPr(result)[1];
    perpFoot.v3 = mxGetPr(result)[2];
    mxDestroyArray(result);
    return perpFoot;
}

int main()
{
    Engine *ep;
/*
 * Call engOpen with a NULL string. This starts a MATLAB process
 * on the current host using the command "matlab".
 */
    if (!(ep = engOpen(""))) {
        fprintf(stderr, "\nCan't start MATLAB engine\n");
        return EXIT_FAILURE;
    }
    struct vector a, u, b, v;
    a.v1 = -7;
    a.v2 = 2;
    a.v3 = -3;
    u.v1 = 0;
    u.v2 = 1;
    u.v3 = 2;
    b.v1 = -3;
    b.v2 = -3;
    b.v3 = 3;
    v.v1 = 1;
    v.v2 = 2;
    v.v3 = 1;
    struct vector oneLine = perpFootOneLine(a, u, b, ep);
    printf("result is [%f, %f, %f]\n", oneLine.v1, oneLine.v2, oneLine.v3);
    struct vector* twoLines = perpFootTwoLines(a, u, b, v, ep);
    printf("Lotfußpunkt von f ist [%f, %f, %f]\n", twoLines[0].v1, twoLines[0].v2, twoLines[0].v3);
    printf("Lotfußpunkt von g ist [%f, %f, %f]\n", twoLines[1].v1, twoLines[1].v2, twoLines[1].v3);
    engClose;
    return EXIT_SUCCESS;
}



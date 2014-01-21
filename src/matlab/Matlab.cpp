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
#include "Vector.h"
#include "Line.h"
#define  BUFSIZE 256

void enterVariablesOneLine(Line f, Vector b, Engine *ep) {
	mxArray *a1, *a2, *a3, *b1, *b2, *b3, *u1, *u2, *u3;

	double dataa1[1] = {f.getA().getV1()};
	double dataa2[1] = {f.getA().getV2()};
	double dataa3[1] = {f.getA().getV3()};

	double datau1[1] = {f.getU().getV1()};
	double datau2[1] = {f.getU().getV2()};
	double datau3[1] = {f.getU().getV3()};

	double datab1[1] = {b.getV1()};
     double datab2[1] = {b.getV2()};
     double datab3[1] = {b.getV3()};

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

void enterVariablesTwoLines(Line f, Line g, Engine *ep) {
     mxArray *a1, *a2, *a3, *b1, *b2, *b3, *u1, *u2, *u3, *v1, *v2, *v3;

     double dataa1[1] = {f.getA().getV1()};
     double dataa2[1] = {f.getA().getV2()};
     double dataa3[1] = {f.getA().getV3()};

     double datau1[1] = {f.getU().getV1()};
     double datau2[1] = {f.getU().getV2()};
     double datau3[1] = {f.getU().getV3()};

     double datab1[1] = {g.getA().getV1()};
     double datab2[1] = {g.getA().getV2()};
     double datab3[1] = {g.getA().getV3()};

     double datav1[1] = {g.getU().getV1()};
     double datav2[1] = {g.getU().getV2()};
     double datav3[1] = {g.getU().getV3()};

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

Vector perpFootOneLine(Line f, Vector b, Engine *ep) {
    enterVariablesOneLine(f, b, ep);
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
        engEvalString(ep, "product = dot(u,vector)");
        engEvalString(ep, "r = solve(product == d, r)");
        engEvalString(ep, "x = cast(r, 'double');");
    */

    // as u is the direction vector it can't be 0
    engEvalString(ep, "x = (a1*u1+a2*u2+a3*u3-d)/(-u1*u1-u2*u2-u3*u3)");
    engEvalString(ep, "result = a + x*u");

    result = engGetVariable(ep, "result");
    //printf("result is [%f, %f, %f]\n", mxGetPr(result)[0], mxGetPr(result)[1], mxGetPr(result)[2]);
    Vector* perpFoot = new Vector(mxGetPr(result)[0], mxGetPr(result)[1], mxGetPr(result)[2]);
    mxDestroyArray(result);
    return *perpFoot;
}

/*
  Both Lines are defined by f = a + r*u, g = b + s*v, u and v are already direction vectors.
  You have to creat a Vector** instance, so the result can be saved in it.
  returns 0 if parallel, result isn't changed
	  1 if f and g intersect, result[0] is intersection point
	  2 if f and g are scew, result[0] is perpFoot of f, result[1] is perpFoot of g
*/
int perpFootTwoLines(Line f, Line g, Engine *ep, Vector **result) {
    enterVariablesTwoLines(f, g, ep);
    mxArray *same, *parallel;
    engEvalString(ep, "a = [a1, a2, a3];");
    engEvalString(ep, "u = [u1, u2, u3];");
    engEvalString(ep, "b = [b1, b2, b3];");
    engEvalString(ep, "v = [v1, v2, v3];");
    engEvalString(ep, "dif = [a1 - b1, a2 - b2, a3 - b3]");
    // checks whether the lines intersect, if so result false
    // checks whether a line or a row of A would be 0, so that it can't be inverted
    engEvalString(ep, "bb = [dif(1); dif(2)];");
    // A*x = bb, x = (r, s)
    // checks if equation is also right for the third vectorcomponent.
    if (((f.getU().getV1() != 0) && (g.getU().getV1() != 0)) && ((f.getU().getV2() != 0) && (g.getU().getV2() != 0)) && ((f.getU().getV1() != 0) && (f.getU().getV2() != 0)) && ((g.getU().getV1() != 0) && (g.getU().getV2() != 0))) {
    	engEvalString(ep, "A = [-u1 v1; -u2 v2];");
    	engEvalString(ep, "x = A\\bb");
	engEvalString(ep, "same = (a1+x(1)*u1 == b1 + x(2) * v1)");
    } else if (((f.getU().getV1() != 0) && (g.getU().getV1() != 0)) && ((f.getU().getV3() != 0) && (g.getU().getV3() != 0)) && ((f.getU().getV1() != 0) && (f.getU().getV3() != 0)) && ((g.getU().getV1() != 0) && (g.getU().getV3() != 0))) {
    	engEvalString(ep, "A = [-u1 v1; -u3 v3]");
    	engEvalString(ep, "x = A\\bb");
	engEvalString(ep, "same = (a2+x(1)*u2 == b2 + x(2) * v2)");
    } else {
    	engEvalString(ep, "A = [-u2 v2; -u3 v3]"); 
    	engEvalString(ep, "x = A\\bb");
	engEvalString(ep, "same = (a3+x(1)*u3 == b3 + x(2) * v3)");
    }
    same = engGetVariable(ep, "same");
    if (mxGetPr(same)[0] != 0.0) {
	engEvalString(ep, "i = a + x(1)*u");
	mxArray* intersectionpoint = engGetVariable(ep, "i");
	result[0] = new Vector(mxGetPr(intersectionpoint)[0], mxGetPr(intersectionpoint)[1], mxGetPr(intersectionpoint)[2]);
	mxDestroyArray(same);
	mxDestroyArray(intersectionpoint);
	return 1;
    }
    // cecks whether the lines are parallel: 0 = u + r*v?
    engEvalString(ep, "parallel = (((-u1/v1) == (-u2/v2)) && ((-u1/v1) == (-u3/v3)));");
    parallel = engGetVariable(ep, "parallel");
    if (mxGetPr(parallel)[0] != 0.0) {
	mxDestroyArray(parallel);
	return 0;
    }
    // calculating perpendicular foot points.
    mxArray *resultf, *resultg;
    engEvalString(ep, "n = cross(u, v);");
    engEvalString(ep, "A = [-u1 -n(1) v1; -u2 -n(2) v2; -u3 -n(3) v3];");
    engEvalString(ep, "bb = [dif(1); dif(2); dif(3)];");
    // A*x = bb, x = (r, t, s)
    engEvalString(ep, "x = inv(A)*bb");
    engEvalString(ep, "perpf = a + x(1,1) * u");
    engEvalString(ep, "perpg = b + x(3,1) * v");
    resultf = engGetVariable(ep, "perpf");
    //printf("Lotfußpunkt von f ist [%f, %f, %f]\n", mxGetPr(resultf)[0], mxGetPr(resultf)[1], mxGetPr(resultf)[2]);
    resultg = engGetVariable(ep, "perpg");
    //printf("Lotfußpunkt von g ist [%f, %f, %f]\n", mxGetPr(resultg)[0], mxGetPr(resultg)[1], mxGetPr(resultg)[2]);
    Vector* perpFootf = new Vector(mxGetPr(resultf)[0], mxGetPr(resultf)[1], mxGetPr(resultf)[2]);
    Vector* perpFootg = new Vector(mxGetPr(resultg)[0], mxGetPr(resultg)[1], mxGetPr(resultg)[2]);
    //printf("Lotfußpunkt von g ist [%f, %f, %f]\n", perpFootg->getV1(), perpFootg->getV2(),perpFootg->getV3());
    result[0] = perpFootf;
    result[1] = perpFootg;
    mxDestroyArray(resultf);
    mxDestroyArray(resultg);
    mxDestroyArray(same);
    mxDestroyArray(parallel);
    return 2;
}

Vector interpolateLines(Line *lines, int quantity, Engine *ep) {
    Vector points[2*quantity];
    int pos = 0;
    int intersects;
    Vector* result[2];
    for (int i = 0; i < quantity; i++) {
	for (int j = i + 1; j < quantity; j++) {
		intersects = perpFootTwoLines(lines[i], lines[j], ep, result);
		if (intersects == 1) {
			points[pos] = *result[0];
			pos++;
		} else if (intersects == 2) {
			points[pos] = *result[0];
			pos++;
			points[pos] = *result[1];
			pos++;
		}
	}
    }
    int v1, v2, v3;
    for (int i = 0; i < pos; i++) {
	v1 = v1 + points[pos].getV1();
	v2 = v2 + points[pos].getV2();
	v3 = v3 + points[pos].getV3();
   }
   v1 = v1 / pos;
   v2 = v2 / pos;
   v3 = v3 / pos;
   Vector *approximated = new Vector(v1, v2, v3);
   return *approximated;
}

Vector interpolateLine(Line line, Vector quadPos, double interpolationFactor, Engine *ep) {
	Vector newPos = perpFootOneLine(line, quadPos, ep);
	//interpolatedNewPos = newPos * interpolationFactor + quadPos * (1 - interpolationFactor)
	double v1 = newPos.getV1()*interpolationFactor + quadPos.getV1() * (1 - interpolationFactor);
	double v2 = newPos.getV2()*interpolationFactor + quadPos.getV2() * (1 - interpolationFactor);
	double v3 = newPos.getV3()*interpolationFactor + quadPos.getV3() * (1 - interpolationFactor);
	Vector* interpolatedNewPos = new Vector(v1, v2, v3);
	return *interpolatedNewPos;
}

int main()
{
    Engine *ep;
    // starts a MATLAB process
    if (!(ep = engOpen(""))) {
        fprintf(stderr, "\nCan't start MATLAB engine\n");
        return EXIT_FAILURE;
    }
    
/*
    //Lotfußpunkt (6, 3, 1)
    Vector* a = new Vector(-2, 1, 7);
    Vector* u = new Vector(4, 1, -3);
    Vector* b = new Vector(10, 5, 7);

    //Lotfußpunkt f: (-7,5,3) Lotfußpunkt g: (-1, 1, 5)
    Vector* a = new Vector(-7, 2, -3);
    Vector* u = new Vector(0, 1, 2);
    Vector* b = new Vector(-3, -3, 3);
    Vector* v = new Vector(1, 2, 1);

    //Lotfußpunkt f: (8, -1, 7) Lotfußpunkt g: (8, 11, 1)
    Vector* a = new Vector(3, -1, 7);
    Vector* u = new Vector(1, 0, 0);
    Vector* b = new Vector(2, 8, -5);
    Vector* v = new Vector(2, 1, 2);
*/
    //intersection point (3, 6, 11)
    Vector* a = new Vector(4, 2, 8);
    Vector* u = new Vector(-1, 4, 3);
    Vector* b = new Vector(5, 8, 21);
    Vector* v = new Vector(1, 1, 5);
    Line* f = new Line(a, u);
    Line* g = new Line(b, v);

    Vector oneLine = perpFootOneLine(*f, *b, ep);
    printf("result is [%f, %f, %f]\n", oneLine.getV1(), oneLine.getV2(), oneLine.getV3());

    Vector* result[2];
    int intersects = perpFootTwoLines(*f, *g, ep, result);
    if (intersects == 0) {
	printf("parallel or same\n");
    } else {
	if (intersects == 1) {
    		printf("intersectionpoint is [%f, %f, %f]\n", result[0]->getV1(), result[0]->getV2(), result[0]->getV3());
	} else {
		if (intersects == 2) { 
  	  		printf("Lotfußpunkt von f ist [%f, %f, %f]\n", result[0]->getV1(), result[0]->getV2(), result[0]->getV3());
    			printf("Lotfußpunkt von g ist [%f, %f, %f]\n", result[1]->getV1(), result[1]->getV2(), result[1]->getV3());
		}
	}
    }
    engClose;
    return EXIT_SUCCESS;
}



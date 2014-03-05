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

Matlab::Matlab() {
    Engine *ep;
    // starts a MATLAB process
    if (!(ep = engOpen(""))) {
            fprintf(stderr, "\nCan't start MATLAB engine\n");
    } else {
        this->ep = ep;
    }
}

Matlab::Matlab (Engine *ep) {
    this->ep = ep;
}

void Matlab::destroyMatlab() {
    engClose(ep);
}

Engine* Matlab::getEngine() {
    return this->ep;
}

Vector Matlab::perpFootOneLine(Line f, Vector b) {
    f.getA().putVariable("a", ep);
    f.getU().putVariable("u", ep);
    b.putVariable("b", ep);
    mxArray *result;
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

int Matlab::perpFootTwoLines(Line f, Line g, Vector **result) {
    f.getA().putVariable("a", ep);
    f.getU().putVariable("u", ep);
    g.getA().putVariable("b", ep);
    g.getU().putVariable("v", ep);
    mxArray *same, *parallel;
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

Vector Matlab::interpolateLines(Line *lines, int quantity) {
    // saving perpendicular foot points of all lines of array lines
    Vector *points = new Vector[2*quantity];
	int pos = 0;
	int intersects;
	Vector* result[2];
	for (int i = 0; i < quantity; i++) {
		for (int j = i + 1; j < quantity; j++) {
            intersects = perpFootTwoLines(lines[i], lines[j], result);
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
    // calculating average of all points in array points
    double v1 = 0;
    double v2 = 0;
    double v3 = 0;
    for (int i = 0; i < pos; i++) {
        v1 = v1 + points[i].getV1();
        v2 = v2 + points[i].getV2();
        v3 = v3 + points[i].getV3();
    }
	v1 = v1 / pos;
	v2 = v2 / pos;
	v3 = v3 / pos;
	Vector *approximated = new Vector(v1, v2, v3);
    return *approximated;
}

Vector Matlab::interpolateLine(Line line, Vector quadPos, double interpolationFactor) {
    Vector newPos = perpFootOneLine(line, quadPos);
	double v1 = newPos.getV1()*interpolationFactor + quadPos.getV1() * (1 - interpolationFactor);
	double v2 = newPos.getV2()*interpolationFactor + quadPos.getV2() * (1 - interpolationFactor);
	double v3 = newPos.getV3()*interpolationFactor + quadPos.getV3() * (1 - interpolationFactor);
	Vector* interpolatedNewPos = new Vector(v1, v2, v3);
	return *interpolatedNewPos;
}


Line Matlab::getIntersectionLine(Line f, Vector directV1, Line g, Vector directV2) {
    f.getA().putVariable("a", ep);
    f.getU().putVariable("u", ep);
    Vector v = directV1.add(f.getA().mult(-1));
    v.putVariable("v", ep);
    g.getA().putVariable("b", ep);
    g.getU().putVariable("w", ep);
    // E1 == g
    engEvalString(ep, "A = [u(1) v(1) -w(1); u(2) v(2) -w(2); u(3) v(3) -w(3)]");
    engEvalString(ep, "diff = b - a");
    engEvalString(ep, "bb = [diff(1); diff(2); diff(3)]");
    // x = (r, s, t)
    engEvalString(ep, "x = inv(A) * bb");
    mxArray *x = engGetVariable(ep, "x");
    // point on the intersectionline
    Vector intersection1 = g.getA().add(g.getU().mult(mxGetPr(x)[2]));

    Vector w = directV2.add(g.getA().mult(-1));
    w.putVariable("w", ep);
    // E1 == g.getA() + r * (directV2 - g.getA())
    engEvalString(ep, "A = [u(1) v(1) -w(1); u(2) v(2) -w(2); u(3) v(3) -w(3)]");

    engEvalString(ep, "diff = b - a");
    engEvalString(ep, "bb = [diff(1); diff(2); diff(3)]");

    // x = (r, s, z)
    engEvalString(ep, "x = inv(A) * bb");
    x = engGetVariable(ep, "x");
    // point on the intersectionline
    Vector intersection2 = g.getA().add(w.mult(mxGetPr(x)[2]));

    Line *intersectionLine = new Line(intersection1, (intersection2.add(intersection1.mult(-1))));
    mxDestroyArray(x);
    return *intersectionLine;
}

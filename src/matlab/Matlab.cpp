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
#include <ros/ros.h>
#include <cmath>
#include "Matrix 2x2.h"
#include "profiling.hpp"
#define  BUFSIZE 256

Matlab::Matlab() {
    Engine *ep;
    // starts a MATLAB process
    if (!(ep = engOpen(""))) {
            fprintf(stderr, "\nCan't start MATLAB engine\n");
    } else {
        this->ep = ep;
    }
    error = 0;
}

Matlab::Matlab (Engine *ep) {
    this->ep = ep;
    error = 0;
}

void Matlab::destroyMatlab() {
    // closes matlab engine
    engClose(ep);
}

Engine* Matlab::getEngine() {
    return this->ep;
}

Vector Matlab::perpFootOneLine(Line f, Vector b) {
    Vector a = f.getA();
    Vector u = f.getU();
    // d = b * u
    float d = b.scalarMult(u);
    // x = (a * u - d)/(- u^2)
    int x = (a.scalarMult(u) - d)/(u.scalarMult(u.mult(-1)));
    // result = a + u * ((a - b) * u)/(- u^2))
    Vector result = a.add(u.mult(x));
    return result;
}

int Matlab::perpFootTwoLinesFastCalculation(Line f, Line g, Vector *result) {
    bool same;
    Vector x;
    // Vectors to check later, if the Matrix of two components of both Vectors can be inverted
    Vector f12 = Vector(f.getU().getV1(), f.getU().getV2(), 0);
    Vector f13 = Vector(f.getU().getV1(), 0, f.getU().getV3());
    Vector g12 = Vector(g.getU().getV1(), g.getU().getV2(), 0);
    Vector g13 = Vector(g.getU().getV1(), 0, g.getU().getV3());

    // first checking, whether the lines are parallel
    if (f.getU().isLinearDependent(g.getU())) {
            return 0;
    }
    // aren't parallel, need to check, whether intersect. has to make sure, that A can be inverted.
    // A*x = bb, x = (r, s)
    else if (!(((f.getU().getV1() == 0) && (g.getU().getV1() == 0)) || ((f.getU().getV2() == 0) && (g.getU().getV2() == 0))
                 || ((f.getU().getV1() == 0) && (f.getU().getV2() == 0)) || ((g.getU().getV1() == 0) && (g.getU().getV2() == 0))
                 || (f12.isLinearDependent(g12)))) {
        Matrix2x2 A = Matrix2x2(-f.getU().getV1(), g.getU().getV1(), -f.getU().getV2(), g.getU().getV2());
        Vector bb = f.getA().add(g.getA().mult(-1));
        x = A.inverse().multiplicate(bb);
        // checks if equation is also right for the third vectorcomponent.
        if (fabs(f.getA().getV3() + x.getV1() * f.getU().getV3() - g.getA().getV3() + x.getV2() * g.getU().getV3()) < 0.0000000001) {
            same = true;
        } else {
            same = false;
        }

    } else if (!(((f.getU().getV1() == 0) && (g.getU().getV1() == 0)) || ((f.getU().getV3() == 0) && (g.getU().getV3() == 0))
                 || ((f.getU().getV1() == 0) && (f.getU().getV3() == 0)) || ((g.getU().getV1() == 0) && (g.getU().getV3() == 0))
                 || (f13.isLinearDependent(g13)))) {
        Matrix2x2 A = Matrix2x2(-f.getU().getV1(), g.getU().getV1(), -f.getU().getV3(), g.getU().getV3());
        Vector bb = f.getA().add(g.getA().mult(-1));
        double v2 = bb.getV3();
        bb.setV2(v2);
        x = A.inverse().multiplicate(bb);
        // checks if equation is also right for the third vectorcomponent.
        if (fabs(f.getA().getV2() + x.getV1() * f.getU().getV2() - g.getA().getV2() + x.getV2() * g.getU().getV2()) < 0.0000000001) {
            same = true;
        } else {
            same = false;
        }
    } else {
        Matrix2x2 A = Matrix2x2(-f.getU().getV2(), g.getU().getV2(), -f.getU().getV3(), g.getU().getV3());
        Vector bb = f.getA().add(g.getA().mult(-1));
        double v1 = bb.getV2();
        double v2 = bb.getV3();
        bb.setV1(v1);
        bb.setV2(v2);
        x = A.inverse().multiplicate(bb);
        // checks if equation is also right for the third vectorcomponent.
        if (fabs(f.getA().getV1() + x.getV1() * f.getU().getV1() - g.getA().getV1() + x.getV2() * g.getU().getV1()) < 0.0000000001) {
            same = true;
        } else {
            same = false;
        }
    }

    // if same = true, lines intersect
    if (same) {
        Vector intersectionpoint = f.getA().add(f.getU().mult(x.getV1()));
        result[0] = intersectionpoint;
        return 1;
    }

    // lines are skew, calculating perpendicular foot points.
    Vector n = f.getU().cross(g.getU());
    Matrix A = Matrix(-f.getU().getV1(), -n.getV1(), g.getU().getV1(), -f.getU().getV2(), -n.getV2(), g.getU().getV2(), -f.getU().getV3(), -n.getV3(), g.getU().getV3());
    Vector bb = f.getA().add(g.getA().mult(-1));

    // A*x = bb, x = (r, t, s)
    x = bb.aftermult(A.inverse());
    Vector perpFootf = f.getA().add(f.getU().mult(x.getV1()));
    Vector perpFootg = g.getA().add(g.getU().mult(x.getV3()));
    result[0] = perpFootf;
    result[1] = perpFootg;
    return 2;
}

int Matlab::perpFootTwoLines(Line f, Line g, Vector **result) {

    f.getA().putVariable("a", ep);
    f.getU().putVariable("u", ep);
    g.getA().putVariable("b", ep);
    g.getU().putVariable("v", ep);
    engEvalString(ep, "dif = [a1 - b1, a2 - b2, a3 - b3]");
    mxArray *same;

    // Vectors to check later, if the Matrix of two components of both Vectors can be inverted
    Vector f12 = Vector(f.getU().getV1(), f.getU().getV2(), 0);
    Vector f13 = Vector(f.getU().getV1(), 0, f.getU().getV3());
    Vector g12 = Vector(g.getU().getV1(), g.getU().getV2(), 0);
    Vector g13 = Vector(g.getU().getV1(), 0, g.getU().getV3());

    // first checking, whether the lines are parallel
    if (f.getU().isLinearDependent(g.getU())) {
            return 0;
    }

    // aren't parallel, need to check, whether intersect. has to make sure, that A can be inverted.
    // A*x = bb, x = (r, s)
    else if (!(((f.getU().getV1() == 0) && (g.getU().getV1() == 0)) || ((f.getU().getV2() == 0) && (g.getU().getV2() == 0))
                 || ((f.getU().getV1() == 0) && (f.getU().getV2() == 0)) || ((g.getU().getV1() == 0) && (g.getU().getV2() == 0))
                 || (f12.isLinearDependent(g12)))) {
        engEvalString(ep, "A = [-u1 v1; -u2 v2];");
        engEvalString(ep, "bb = [dif(1); dif(2)];");
        engEvalString(ep, "x = inv(A)*bb");
        // checks if equation is also right for the third vectorcomponent.
        engEvalString(ep, "same = (a3+x(1)*u3 == b3 + x(2) * v3)");
    } else if (!(((f.getU().getV1() == 0) && (g.getU().getV1() == 0)) || ((f.getU().getV3() == 0) && (g.getU().getV3() == 0))
                 || ((f.getU().getV1() == 0) && (f.getU().getV3() == 0)) || ((g.getU().getV1() == 0) && (g.getU().getV3() == 0))
                 || (f13.isLinearDependent(g13)))) {
        engEvalString(ep, "A = [-u1 v1; -u3 v3]");
        engEvalString(ep, "bb = [dif(1); dif(3)];");
        engEvalString(ep, "x = inv(A)*bb");
        // checks if equation is also right for the third vectorcomponent.
		engEvalString(ep, "same = (a2+x(1)*u2 == b2 + x(2) * v2)");
    } else {
        engEvalString(ep, "A = [-u2 v2; -u3 v3]");
        engEvalString(ep, "bb = [dif(2); dif(3)];");
        engEvalString(ep, "x = inv(A)*bb");
        // checks if equation is also right for the third vectorcomponent.
        engEvalString(ep, "same = (a1+x(1)*u1 == b1 + x(2) * v1)");
    }

    // if same = 0, lines intersect
	same = engGetVariable(ep, "same");
	if (mxGetPr(same)[0] != 0.0) {
		engEvalString(ep, "i = a + x(1)*u");
		mxArray* intersectionpoint = engGetVariable(ep, "i");
		result[0] = new Vector(mxGetPr(intersectionpoint)[0], mxGetPr(intersectionpoint)[1], mxGetPr(intersectionpoint)[2]);
		mxDestroyArray(same);
		mxDestroyArray(intersectionpoint);
		return 1;
	}

    // lines are skew, calculating perpendicular foot points.
	mxArray *resultf, *resultg;
	engEvalString(ep, "n = cross(u, v);");
	engEvalString(ep, "A = [-u1 -n(1) v1; -u2 -n(2) v2; -u3 -n(3) v3];");
	engEvalString(ep, "bb = [dif(1); dif(2); dif(3)];");

    // A*x = bb, x = (r, t, s)
    engEvalString(ep, "x = inv(A)*bb");
	engEvalString(ep, "perpf = a + x(1,1) * u");
	engEvalString(ep, "perpg = b + x(3,1) * v");

    resultf = engGetVariable(ep, "perpf");
    resultg = engGetVariable(ep, "perpg");
    Vector* perpFootf = new Vector(mxGetPr(resultf)[0], mxGetPr(resultf)[1], mxGetPr(resultf)[2]);
	Vector* perpFootg = new Vector(mxGetPr(resultg)[0], mxGetPr(resultg)[1], mxGetPr(resultg)[2]);

    result[0] = perpFootf;
    result[1] = perpFootg;

    mxDestroyArray(resultf);
    mxDestroyArray(resultg);
    mxDestroyArray(same);

    return 2;
}

Vector Matlab::interpolateLines(Line *lines, int quantity, Vector oldPos, double interpolationFactor) {

    int sum = 0;
    for (int i = 1; i < quantity; i++) {
        sum += i;
    }

    // saving perpendicular foot points of all lines of array lines
    Vector *points = new Vector[2 * sum];
	int pos = 0;
    int intersects;
    Vector result[2];

    // calculating perpendicular foot points/intersection points
	for (int i = 0; i < quantity; i++) {
		for (int j = i + 1; j < quantity; j++) {

            intersects = perpFootTwoLinesFastCalculation(lines[i], lines[j], result);

            if (intersects == 1) {
                // lines interct
                points[pos] = result[0];
                pos++;
            } else if (intersects == 2) {
                // lines are skew
                points[pos] = result[0];
                pos++;
                points[pos] = result[1];
                pos++;
			}
		}
    }

    error = 0;

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

    Vector perp = Vector(v1, v2, v3);
    for (int i = 0; i < pos; i++) {
        ROS_DEBUG("perp is [%f, %f, %f]", points[i].getV1(), points[i].getV2(), points[i].getV3());
        error += points[i].add(perp.mult(-1)).getLength();
    }

    error = error / pos;

    // interpolating between last seen position and new calculated position
    perp = interpolate(oldPos, perp, interpolationFactor);

    return perp;
}

Vector Matlab::interpolateLine(Line line, Vector quadPos, double interpolationFactor) {

    // caculate perpendicular foor point of line and quadPos
    Vector newPos = perpFootOneLine(line, quadPos);

    // interpolating between last seen position and new calculated position
    Vector result = interpolate(quadPos, newPos, interpolationFactor);

    error = newPos.add(result.mult(-1)).getLength();

    return result;
}

Vector Matlab::interpolate(Vector oldPos, Vector newPos, double interpolationFactor) {
    double v1 = newPos.getV1()*interpolationFactor + oldPos.getV1() * (1 - interpolationFactor);
    double v2 = newPos.getV2()*interpolationFactor + oldPos.getV2() * (1 - interpolationFactor);
    double v3 = newPos.getV3()*interpolationFactor + oldPos.getV3() * (1 - interpolationFactor);
    Vector result(v1, v2, v3);
    return result;
}

Line Matlab::getIntersectionLine(Line f, Vector directV1, Line g, Vector directV2) {

    f.getA().putVariable("a", ep);
    f.getU().putVariable("u", ep);
    g.getA().putVariable("b", ep);
    g.getU().putVariable("w", ep);

    // v = directV1 - a, is second direction vector of first plain
    Vector v = directV1.add(f.getA().mult(-1));
    v.putVariable("v", ep);

    std::string input;
    // E1 == g
    input = "A = [u(1) v(1) -w(1); u(2) v(2) -w(2); u(3) v(3) -w(3)];";
    input += "diff = b - a;";
    input += "bb = [diff(1); diff(2); diff(3)];";
    // x = (r, s, t)
    input += "x = inv(A) * bb;";
    // enter resulting string in matlab
    engEvalString(ep, input.c_str());
    mxArray *x = engGetVariable(ep, "x");
    // point on the intersectionline
    Vector intersection1 = g.getA().add(g.getU().mult(mxGetPr(x)[2]));

    // w = directV2 - b, is second direction vector of second plain
    Vector w = directV2.add(g.getA().mult(-1));
    w.putVariable("w", ep);
    engEvalString(ep, input.c_str());
    x = engGetVariable(ep, "x");
    // point on the intersectionline
    Vector intersection2 = g.getA().add(w.mult(mxGetPr(x)[2]));

    // intersection line is line through both points
    Line intersectionLine = Line(intersection1, (intersection2.add(intersection1.mult(-1))));
    mxDestroyArray(x);
    return intersectionLine;
}

Line Matlab::getIntersectionLineFastCalculation(Line f, Vector directV1, Line g, Vector directV2) {

    // E1 == g
    Vector v = directV1.add(f.getA().mult(-1));
    Matrix A = Matrix(f.getU().getV1(), v.getV1(), -g.getU().getV1(), f.getU().getV2(), v.getV2(), -g.getU().getV2(), f.getU().getV3(), v.getV3(), -g.getU().getV3());
    Vector diff = g.getA().add(f.getA().mult(-1));

    // x = (r, s, t)
    Vector x = diff.aftermult(A.inverse());
    Vector intersection1 = g.getA().add(g.getU().mult(x.getV3()));

    // w = directV2 - g.getA(), is second direction vector of second plain
    Vector w = directV2.add(g.getA().mult(-1));
    A = Matrix(f.getU().getV1(), v.getV1(), -w.getV1(), f.getU().getV2(), v.getV2(), -w.getV2(), f.getU().getV3(), v.getV3(), -w.getV3());
    x = diff.aftermult(A.inverse());
    Vector intersection2 = g.getA().add(w.mult(x.getV3()));

    // intersection line is line through both points
    Line intersectionLine = Line(intersection1, (intersection2.add(intersection1.mult(-1))));
    return intersectionLine;
}

double Matlab::getError() {
    return this->error;
}

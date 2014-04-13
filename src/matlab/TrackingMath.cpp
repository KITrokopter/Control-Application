/*
 * TrackingMath.cpp
 *
 *  Created on: 13.01.2014
 *      Author: daniela
 */

#include "TrackingMath.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <ros/ros.h>
#include <cmath>
#include "Matrix2x2.h"
#include "profiling.hpp"
#define  BUFSIZE 256

TrackingMath::TrackingMath()
{
	error = 0;
}

Vector TrackingMath::perpFootOneLine(Line f, Vector b)
{
	Vector a = f.getA();
	Vector u = f.getU();
	// d = b * u
	float d = b.scalarMult(u);
	// x = (a * u - d)/(- u^2)
	int x = (a.scalarMult(u) - d) / (u.scalarMult(u.mult(-1)));
	// result = a + u * ((a - b) * u)/(- u^2))
	Vector result = a.add(u.mult(x));
	return result;
}

int TrackingMath::perpFootTwoLines(Line f, Line g, Vector *result)
{
	bool same;
	Vector x;
	// Vectors to check later, if the Matrix of two components of both Vectors
	// can be inverted
	Vector f12 = Vector(f.getU().getV1(), f.getU().getV2(), 0);
	Vector f13 = Vector(f.getU().getV1(), 0, f.getU().getV3());
	Vector g12 = Vector(g.getU().getV1(), g.getU().getV2(), 0);
	Vector g13 = Vector(g.getU().getV1(), 0, g.getU().getV3());

	// first checking, whether the lines are parallel
	if (f.getU().isLinearDependent(g.getU())) {
		return 0;
	}
	// aren't parallel, need to check, whether intersect. has to make sure, that
	// A can be inverted.
	// A*x = bb, x = (r, s)
	else if (!(((f.getU().getV1() == 0) &&
	            (g.getU().getV1() == 0)) || ((f.getU().getV2() == 0) && (g.getU().getV2() == 0))
	           || ((f.getU().getV1() == 0) &&
	               (f.getU().getV2() == 0)) || ((g.getU().getV1() == 0) && (g.getU().getV2() == 0))
	           || (f12.isLinearDependent(g12)))) {
		Matrix2x2 A = Matrix2x2(-f.getU().getV1(), g.getU().getV1(), -f.getU().getV2(), g.getU().getV2());
		Vector bb = f.getA().add(g.getA().mult(-1));
		x = A.inverse().multiplicate(bb);
		// checks if equation is also right for the third vectorcomponent.
		if (fabs(f.getA().getV3() + x.getV1() * f.getU().getV3() - g.getA().getV3() + x.getV2() * g.getU().getV3()) <
		    0.0000000001) {
			same = true;
		} else {
			same = false;
		}
	} else if (!(((f.getU().getV1() == 0) &&
	              (g.getU().getV1() == 0)) || ((f.getU().getV3() == 0) && (g.getU().getV3() == 0))
	             || ((f.getU().getV1() == 0) &&
	                 (f.getU().getV3() == 0)) || ((g.getU().getV1() == 0) && (g.getU().getV3() == 0))
	             || (f13.isLinearDependent(g13)))) {
		Matrix2x2 A = Matrix2x2(-f.getU().getV1(), g.getU().getV1(), -f.getU().getV3(), g.getU().getV3());
		Vector bb = f.getA().add(g.getA().mult(-1));
		double v2 = bb.getV3();
		bb.setV2(v2);
		x = A.inverse().multiplicate(bb);
		// checks if equation is also right for the third vectorcomponent.
		if (fabs(f.getA().getV2() + x.getV1() * f.getU().getV2() - g.getA().getV2() + x.getV2() * g.getU().getV2()) <
		    0.0000000001) {
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
		if (fabs(f.getA().getV1() + x.getV1() * f.getU().getV1() - g.getA().getV1() + x.getV2() * g.getU().getV1()) <
		    0.0000000001) {
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
	Matrix A = Matrix(-f.getU().getV1(), -n.getV1(), g.getU().getV1(), -f.getU().getV2(), -n.getV2(),
	                  g.getU().getV2(), -f.getU().getV3(), -n.getV3(), g.getU().getV3());
	Vector bb = f.getA().add(g.getA().mult(-1));

	// A*x = bb, x = (r, t, s)
	x = bb.aftermult(A.inverse());
	Vector perpFootf = f.getA().add(f.getU().mult(x.getV1()));
	Vector perpFootg = g.getA().add(g.getU().mult(x.getV3()));
	result[0] = perpFootf;
	result[1] = perpFootg;
	return 2;
}

Vector TrackingMath::interpolateLines(Line *lines, int quantity, Vector oldPos, double interpolationFactor)
{
	int sum = 0;
	for (int i = 1; i < quantity; i++) {
		sum += i;
	}

	// saving perpendicular foot points of all lines of array lines
	Vector points[2 * sum];
	int pos = 0;
	int intersects;
	Vector result[2];

	// calculating perpendicular foot points/intersection points
	for (int i = 0; i < quantity; i++) {
		for (int j = i + 1; j < quantity; j++) {
			// only interpolates if angle is bigger than pi/8
			if (getAngle(lines[i].getU(), lines[j].getU()) > (M_PI / 8.0)) {
				intersects = perpFootTwoLines(lines[i], lines[j], result);

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
	}

	if (pos == 0) {
		return Vector(false);
	} else {
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
			error += points[i].add(perp.mult(-1)).getLength();
		}

		error = error / pos;

		// interpolating between last seen position and new calculated position
		perp = interpolate(oldPos, perp, interpolationFactor);

		return perp;
	}
}

Vector TrackingMath::interpolateLine(Line line, Vector quadPos, double interpolationFactor)
{
	// caculate perpendicular foor point of line and quadPos
	Vector newPos = perpFootOneLine(line, quadPos);

	// interpolating between last seen position and new calculated position
	Vector result = interpolate(quadPos, newPos, interpolationFactor);

	error = newPos.add(result.mult(-1)).getLength();

	return result;
}

Vector TrackingMath::interpolate(Vector oldPos, Vector newPos, double interpolationFactor)
{
	double v1 = newPos.getV1() * interpolationFactor + oldPos.getV1() * (1 - interpolationFactor);
	double v2 = newPos.getV2() * interpolationFactor + oldPos.getV2() * (1 - interpolationFactor);
	double v3 = newPos.getV3() * interpolationFactor + oldPos.getV3() * (1 - interpolationFactor);
	Vector result(v1, v2, v3);
	return result;
}

Line TrackingMath::getIntersectionLine(Line f, Vector directV1, Line g, Vector directV2)
{
	// E1 == g
	Vector v = directV1.add(f.getA().mult(-1));
	Matrix A = Matrix(f.getU().getV1(), v.getV1(), -g.getU().getV1(), f.getU().getV2(), v.getV2(),
	                  -g.getU().getV2(), f.getU().getV3(), v.getV3(), -g.getU().getV3());
	Vector diff = g.getA().add(f.getA().mult(-1));

	// x = (r, s, t)
	Vector x = diff.aftermult(A.inverse());
	Vector intersection1 = g.getA().add(g.getU().mult(x.getV3()));

	// w = directV2 - g.getA(), is second direction vector of second plain
	Vector w = directV2.add(g.getA().mult(-1));
	A = Matrix(f.getU().getV1(), v.getV1(), -w.getV1(), f.getU().getV2(), v.getV2(), -w.getV2(),
	           f.getU().getV3(), v.getV3(), -w.getV3());
	x = diff.aftermult(A.inverse());
	Vector intersection2 = g.getA().add(w.mult(x.getV3()));

	// intersection line is line through both points
	Line intersectionLine = Line(intersection1, (intersection2.add(intersection1.mult(-1))));
	return intersectionLine;
}

double TrackingMath::getError()
{
	return this->error;
}

double TrackingMath::getAngle(Vector u, Vector v)
{
	// cos(alpha)= u*v/(|u|*|v|)
	double angle = u.scalarMult(v) / (u.getLength() * v.getLength());
	angle = acos(angle);

	// checks whether angle is between 0 and 90 degree
	if (angle > M_PI / 2) {
		angle = -(angle - M_PI);
	}
	return angle;
}

/*
 * calculates the distance of E: a + r*u + s*v and a point x
 */
double TrackingMath::getDistPointPlane(Vector a, Vector u, Vector v, Vector x)
{
	Vector n = Vector(u.getV2() * v.getV3() - u.getV3() * v.getV2(),
	                  u.getV3() * v.getV1() - u.getV1() * v.getV3(), u.getV1() * v.getV2() - u.getV2() * v.getV1());
	double l = n.getLength();
	double result = (n.scalarMult(x) - n.scalarMult(a)) / l;
	if (result < 0) {
		return -result;
	} else {
		return result;
	}
}

/*
 * calculates perpendicular point of point x and plane E: a + ru + sv
 */
Vector TrackingMath::getPerpPointPlane(Vector a, Vector u, Vector v, Vector x)
{
	// n * x = n * a => as x + t*n is the perpendicular point it has to be: n *
	// (x + t*n) = n * a <=> = n*a/(n*x*n.getLength())
	Vector n = Vector(u.getV2() * v.getV3() - u.getV3() * v.getV2(),
	                  u.getV3() * v.getV1() - u.getV1() * v.getV3(), u.getV1() * v.getV2() - u.getV2() * v.getV1());
	double d = n.scalarMult(a) - (n.scalarMult(x));
	double e = n.getV1() * n.getV1() + n.getV2() * n.getV2() + n.getV3() * n.getV3();
	double t = (d / e);
	Vector perp = x.add(n.mult(t));
	return perp;
}


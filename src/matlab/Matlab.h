/*
 * Matlab.h
 *
 *  Created on: 13.01.2014
 *      Author: daniela
 */

#ifndef MATLAB_H_
#define MATLAB_H_
#include "Line.h"
#include "Vector.h"
#include "engine.h"

class Matlab {
private:

    /**
     * error is average distance of perpendicular foot points to interpolated point
     */
    double error;

public:

    /**
     * empty constructor.
     */
	Matlab();

    /**
     * calculates perpendicular foot of line f and point b.
     * @param f Line where the return shoul lie on
     * @param b point
     * @return perpendicular foot of line f and point b
     */
	Vector perpFootOneLine(Line f, Vector b);

    /**
     * calculates perpendicular foot of line f and line g. Result is saved in result
     * @param f Line where the first perpendicular foot should lie on
     * @param g Line where the second perpendicular foot should lie on
     * @param result array of vectors where the perpendicular points should be saved
     * @return 0 if lines are parallel, result isn't changed
     *         1 if f and g intersects, result[0] is the intersection point
     *         2 if f and g are scew, result[0] and result[1] are the perpendicular foot
     */
    int perpFootTwoLines(Line f, Line g, Vector *result);

    /**
     * linear interpolation of some lines
     * @param lines array of all lines that should be interpolated
     * @param quantity number of lines in the array lines
     * @param oldPos is last seen position of quadcopter
     * @param interpolationFactor is factor for interpolation
     * @return vector that is nearest to all lines
     */
    Vector interpolateLines(Line *lines, int quantity, Vector oldPos, double interpolationFactor);

    /**
     * linear interpolation of a point and a line with a interpolationfactor
     * @param line line of a camera pointing to the actual quadcopter position
     * @param quadPos last position of quadcopter
     * @param interpolationFactor
     * @return linear interpolation of line and quadPos
     */
    Vector interpolateLine(Line line, Vector quadPos, double interpolationFactor);

    /**
     * Warning: Only working, if line g and E1 or line g.getA() + r * (directV2-g.getA()) intersects!!
     * calculates intersection line of two plains
     * @param f Line, that is positionated in plane E1
     * @param directV1 Point, that is not in line f and is on the plane E1
     * E1 = f.getA() + r*f.getU() + s*(directV1 - f.getA())
     * @param g Line that is positionated in plane E2
     * @param directV2 point, that is not in line g and is on the plane E2
     * E2 = g.getA() + t* g.getU() + z * (directV2-g.getA())
     * @return intersection line of E1 and E2
     */
	Line getIntersectionLine(Line f, Vector directV1, Line g, Vector directV2);

    /**
     * error distance
     * @return average distance of perpendicular foot points to interpolated point
     */
    double getError();

    /**
     * interpolating
     * @param oldPos old position
     * @param newPos new position
     * @param interpolationFactor
     * @return interpolated new position
     */
    Vector interpolate(Vector oldPos, Vector newPos, double interpolationFactor);

    /**
     * calculating angle between vector u and vector v.
     * @param u first vector
     * @param v second vector
     * @return angle
     */
    double getAngle(Vector u, Vector v);
};

#endif /* MATLAB_H_ */

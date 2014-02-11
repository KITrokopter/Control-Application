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
private: Engine *ep;
public:
	Matlab();
	Matlab(Engine *ep);
	Engine* getEngine();
	void destroyMatlab();
	Vector perpFootOneLine(Line f, Vector b);
	int perpFootTwoLines(Line f, Line g, Vector **result);
    Vector interpolateLines(Line *lines, int quantity);
	Vector interpolateLine(Line line, Vector quadPos, double interpolationFactor);
	Line getIntersectionLine(Line f, Vector directV1, Line g, Vector directV2);
};

#endif /* MATLAB_H_ */

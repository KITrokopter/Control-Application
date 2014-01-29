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
public:
	Matlab();
	void enterVariablesOneLine(Line f, Vector b, Engine *ep);
	void enterVariablesTwoLines(Line f, Line g, Engine *ep);
	Vector perpFootOneLine(Line f, Vector b, Engine *ep);
	int perpFootTwoLines(Line f, Line g, Engine *ep, Vector **result);
	Vector interpolateLines(Line *lines, int quantity, Engine *ep);
	Vector interpolateLine(Line line, Vector quadPos, double interpolationFactor, Engine *ep);
};

#endif /* MATLAB_H_ */

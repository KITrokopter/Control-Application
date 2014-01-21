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
	void enterVariablesOneLine(Line f, Vector b, Engine *ep);
	void enterVariablesTwoLines(Line f, Line g, Engine *ep);
	Vector perpFootOneLine(Line f, Vector b, Engine *ep);
	int perpFootTwoLines(Line f, Line g, Engine *ep, Vector **result);
	Vector getApproximationPoint(Line *lines, int quantity, Engine *ep);
};

#endif /* MATLAB_H_ */

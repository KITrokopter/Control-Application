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
	void enterVariablesOneLine(Line f, Vector b);
	void enterVariablesTwoLines(Line f, Line g);
	Vector perpFootOneLine(Line f, Vector b);
	int perpFootTwoLines(Line f, Line g, Vector **result);
	Vector interpolateLines(Line *lines, int quantity);
	Vector interpolateLine(Line line, Vector quadPos, double interpolationFactor);
};

#endif /* MATLAB_H_ */

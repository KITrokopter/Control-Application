/*
 * Vector.h
 *
 *  Created on: 18.01.2014
 *      Author: daniela
 */

#ifndef VECTOR_H_
#define VECTOR_H_

#include "engine.h"
#include <string>
#include "Matrix.h"


class Vector {
private: double v1, v2, v3;
public:
	Vector(double v1, double v2, double v3);
	Vector();
	double getV1();
	double getV2();
	double getV3();
	void setV1(double v1);
	void setV2(double v2);
	void setV3(double v3);
	Vector add(Vector a);
	Vector mult(double a);
	// calculates Vector * Matrix
	Vector premult(Matrix A);
	// calculates Matrix * Vector
	Vector aftermult(Matrix A);
	int scalarMult(Vector a);
    	double getLength();
    	void putVariable(std::string a, Engine *ep);
	Vector cross(Vector v);
	bool equals(Vector v);
	std::string toString();
	
	/**
	 * Returns true if the vector contains no NaN values.
	 * @return True if the vector contains no NaN values.
	 */
	bool isValid();
};

#endif /* VECTOR_H_ */

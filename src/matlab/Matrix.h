/*
 * Matrix.h
 *
 *  Created on: 18.01.2014
 *      Author: daniela
 */

#ifndef MATRIX_H_
#define MATRIX_H_
#include "Vector.h"


class Matrix {
private: double m11, m12, m13, m21, m22, m23, m31, m32, m33;
public:
	Matrix(double m11, double m12, double m13, double m21, double m22, double m23, double m31, double m32, double m33);
	Matrix();
	// calculates this * a
	Vector mult(Vector a);
};

#endif /* MATRIX_H_ */

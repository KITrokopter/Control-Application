/*
 * Matrix2x2.h
 *
 *  Created on: 22.03.2014
 *      Author: daniela
 */

#ifndef MATRIX2x2_H_
#define MATRIX2x2_H_

#include "Vector.h"

class Matrix2x2 {
private:
    /**
     * values of the Matrix, mij is value of i. line and j. column
     */
    double m11, m12, m21, m22;

public:
    /**
     * empty constructor.
     */
    Matrix2x2();

    /**
     * constructor.
     * @param m11 first row, first column
     * @param m12 first row, second column
     * @param m21 second row, first column
     * @param m22 second row, second column
     */
    Matrix2x2(double m11, double m12, double m21, double m22);

    /**
     * getter.
     * @return first row, first column
     */
    double getM11();

    /**
     * getter.
     * @return first row, second column
     */
	double getM12();

    /**
     * getter.
     * @return second row, first column
     */
	double getM21();

    /**
     * getter.
     * @return second row, second column
     */
	double getM22();

    /**
     * print matrix in three lines
     */
	void printMatrix();

    /**
     * matrix multiplication
     * @param v factor
     * @return Matrix * v, result is Vector with third component = 0
     */
    Vector multiplicate(Vector v);

    /**
     * calculates the determinant
     * @return
     */
    double determinant();

    /**
     * calculates inverse of Matrix
     * @return inverse
     */
    Matrix2x2 inverse();

};

#endif /* MATRIX2x2_H_ */

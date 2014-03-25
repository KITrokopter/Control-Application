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
private:
    /**
     * x-, y- and z-value of vector
     */
    double v1, v2, v3;

    bool valid;
public:
    /**
     * empty constructor.
     */
    Vector();

    /**
     * constructor.
     * @param v1 is x-value
     * @param v2 is y-value
     * @param v3 is z-value
     */
	Vector(double v1, double v2, double v3);

    /**
     * constructor.
     * @param valid whether vector is valid or not.
     */
    Vector(bool valid);

    /**
     * getter.
     * @return x-value
     */
    double getV1();

    /**
     * getter.
     * @return y-value
     */
	double getV2();

    /**
     * getter.
     * @return z-value
     */
	double getV3();

    /**
     * setter.
     * @param v1 x-value
     */
	void setV1(double v1);

    /**
     * setter.
     * @param v2 y-value
     */
	void setV2(double v2);

    /**
     * setter.
     * @param v3 z-value
     */
	void setV3(double v3);

    /**
     * addition of this and a.
     * @param a summand
     * @return sum
     */
	Vector add(Vector a);

    /**
     * multiplication of this and a double.
     * @param a factor
     * @return product
     */
    Vector mult(double a);

    /**
     * calculates Vector * Matrix.
     * @param A Factor
     * @return this * A
     */
	Vector premult(Matrix A);

    /**
     * calculates Matrix * Vector.
     * @param A Factor
     * @return A * this
     */
    Vector aftermult(Matrix A);

    /**
     * calculates scalar multiplication of two Vectors.
     * @param a Factor
     * @return this * a
     */
    double scalarMult(Vector a);

    /**
     * calculates length of vector.
     * @return length of this
     */
    double getLength();

    /**
     * put vector as variable in Matlab engine.
     * @param a name of variable in Matlab
     * @param ep matlab engine
     */
    void putVariable(std::string a, Engine *ep);

    /**
     * calculates cross product of two vectors.
     * @param v Factor
     * @return v x this
     */
	Vector cross(Vector v);

    /**
     * changes Vector in String.
     * @return v1/v2/v3
     */
	std::string toString();

    /**
     * checks linear dependence of two vectors.
     * @param u vector to check with
     * @return true if this and u are linear dependent, false otherwise
     */
	bool isLinearDependent(Vector u);

	/**
	 * Returns true if the vector contains no NaN values.
	 * @return True if the vector contains no NaN values.
	 */
	bool isValid();

    /**
     * getter.
     * @return whether vector is valid or not.
     */
    bool getValid();
};

#endif /* VECTOR_H_ */

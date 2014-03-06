/*
 * Matrix.cpp
 *
 *  Created on: 18.01.2014
 *      Author: daniela
 */

#include "Vector.h"
#include "Matrix.h"

Matrix::Matrix() {
    this->m11 = 0;
    this->m12 = 0;
    this->m13 = 0;
    this->m21 = 0;
    this->m22 = 0;
    this->m23 = 0;
    this->m31 = 0;
    this->m32 = 0;
    this->m33 = 0;
}

Matrix::Matrix(double m11, double m12, double m13, double m21, double m22, double m23, double m31, double m32, double m33) {
    this->m11 = m11;
    this->m12 = m12;
    this->m13 = m13;
    this->m21 = m21;
    this->m22 = m22;
    this->m23 = m23;
    this->m31 = m31;
    this->m32 = m32;
    this->m33 = m33;
}

Vector Matrix::mult(Vector a) {
    double v1 = m11 * a.getV1() + m12 * a.getV2() + m13 * a.getV3();
    double v2 = m21 * a.getV1() + m22 * a.getV2() + m23 * a.getV3();
    double v3 = m31 * a.getV1() + m32 * a.getV2() + m33 * a.getV3();
    Vector *product = new Vector(v1, v2, v3);
    return *product;
}

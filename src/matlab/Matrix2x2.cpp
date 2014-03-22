/*
 * Matrix2x2.cpp
 *
 *  Created on: 22.03.2014
 *      Author: daniela
 */

#include "Matrix2x2.h"
#include <stdio.h>

Matrix2x2::Matrix2x2() {
    this->m11 = 0;
    this->m12 = 0;
    this->m21 = 0;
    this->m22 = 0;
}

Matrix2x2::Matrix2x2(double m11, double m12, double m21, double m22) {
    this->m11 = m11;
    this->m12 = m12;
    this->m21 = m21;
    this->m22 = m22;
}

double Matrix2x2::getM11() {
    return this->m11;
}

double Matrix2x2::getM12() {
    return this->m12;
}

double Matrix2x2::getM21() {
    return this->m21;
}

double Matrix2x2::getM22() {
    return this->m22;
}

void Matrix2x2::printMatrix() {
    printf("%f, %f\n%f, %f\n", m11, m12, m21, m22);
}

Vector Matrix2x2::multiplicate(Vector v) {
    double v1 = m11 * v.getV1() + m12 * v.getV2();
    double v2 = m21 * v.getV1() + m22 * v.getV2();
    return Vector(v1, v2, 0);
}

double Matrix2x2::determinant() {
    return (m11*m22 - m12*m21);
}

Matrix2x2 Matrix2x2::inverse() {
    double det = 1/determinant();
    double i11 = det * m22;
    double i12 = det * -m12;
    double i21 = det * -m21;
    double i22 = det * m11;
    return Matrix2x2(i11, i12, i21, i22);
}

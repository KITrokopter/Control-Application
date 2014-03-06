/*
 * Matrix.cpp
 *
 *  Created on: 18.01.2014
 *      Author: daniela
 */

#include "Matrix.h"
#include "Vector.h"

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

double Matrix::getM11() {
    return this->m11;
}

double Matrix::getM12() {
    return this->m12;
}

double Matrix::getM13() {
    return this->m13;
}

double Matrix::getM21() {
    return this->m21;
}

double Matrix::getM22() {
    return this->m22;
}

double Matrix::getM23() {
    return this->m23;
}

double Matrix::getM31() {
    return this->m31;
}

double Matrix::getM32() {
    return this->m32;
}

double Matrix::getM33() {
    return this->m33;
}

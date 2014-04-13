/*
 * Matrix.cpp
 *
 *  Created on: 18.01.2014
 *      Author: daniela
 */

#include "Matrix.h"
#include <stdio.h>

Matrix::Matrix()
{
	this->m11 = 0;
	this->m12 = 0;
	this->m13 = 0;
	this->m21 = 0;
	this->m22 = 0;
	this->m23 = 0;
	this->m31 = 0;
	this->m32 = 0;
	this->m33 = 0;
	valid = true;
}

Matrix::Matrix(bool valid)
{
	valid = valid;
}

Matrix::Matrix(double m11, double m12, double m13, double m21, double m22, double m23, double m31, double m32,
               double m33)
{
	this->m11 = m11;
	this->m12 = m12;
	this->m13 = m13;
	this->m21 = m21;
	this->m22 = m22;
	this->m23 = m23;
	this->m31 = m31;
	this->m32 = m32;
	this->m33 = m33;
	valid = true;
}

bool Matrix::getValid()
{
	return this->valid;
}

double Matrix::getM11()
{
	return this->m11;
}

double Matrix::getM12()
{
	return this->m12;
}

double Matrix::getM13()
{
	return this->m13;
}

double Matrix::getM21()
{
	return this->m21;
}

double Matrix::getM22()
{
	return this->m22;
}

double Matrix::getM23()
{
	return this->m23;
}

double Matrix::getM31()
{
	return this->m31;
}

double Matrix::getM32()
{
	return this->m32;
}

double Matrix::getM33()
{
	return this->m33;
}

void Matrix::printMatrix()
{
	printf("%f, %f, %f\n%f, %f, %f\n%f, %f, %f\n", m11, m12, m13, m21, m22, m23, m31, m32, m33);
}

Matrix Matrix::multiplicate(Matrix A)
{
	double r11 = m11 * A.getM11() + m12 * A.getM21() + m13 * A.getM31();
	double r12 = m11 * A.getM12() + m12 * A.getM22() + m13 * A.getM32();
	double r13 = m11 * A.getM13() + m12 * A.getM23() + m13 * A.getM33();
	double r21 = m21 * A.getM11() + m22 * A.getM21() + m23 * A.getM31();
	double r22 = m21 * A.getM12() + m22 * A.getM22() + m23 * A.getM32();
	double r23 = m21 * A.getM13() + m22 * A.getM23() + m23 * A.getM33();
	double r31 = m31 * A.getM11() + m32 * A.getM21() + m33 * A.getM31();
	double r32 = m31 * A.getM12() + m32 * A.getM22() + m33 * A.getM32();
	double r33 = m31 * A.getM13() + m32 * A.getM23() + m33 * A.getM33();
	return Matrix(r11, r12, r13, r21, r22, r23, r31, r32, r33);
}

double Matrix::determinant()
{
	double d = m11 * m22 * m33 + m12 * m23 * m31 + m13 * m21 * m32 - m31 * m22 * m13 - m32 * m23 * m11 - m33 * m21 *
	           m12;
	return d;
}

double Matrix::determinant2x2(double a, double b, double c, double d)
{
	return (a * d - b * c);
}

Matrix Matrix::inverse()
{
	double det = 1 / determinant();
	double i11 = det * determinant2x2(m22, m23, m32, m33);
	double i12 = det * determinant2x2(m13, m12, m33, m32);
	double i13 = det * determinant2x2(m12, m13, m22, m23);
	double i21 = det * determinant2x2(m23, m21, m33, m31);
	double i22 = det * determinant2x2(m11, m13, m31, m33);
	double i23 = det * determinant2x2(m13, m11, m23, m21);
	double i31 = det * determinant2x2(m21, m22, m31, m32);
	double i32 = det * determinant2x2(m12, m11, m32, m31);
	double i33 = det * determinant2x2(m11, m12, m21, m22);
	return Matrix(i11, i12, i13, i21, i22, i23, i31, i32, i33);
}


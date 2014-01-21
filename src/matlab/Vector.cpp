/*
 * Vector.cpp
 *
 *  Created on: 18.01.2014
 *      Author: daniela
 */

#include "Vector.h"

Vector::Vector() {
	this->v1= 0;
	this->v2 = 0;
	this->v3 = 0;
}

Vector::Vector(double v1, double v2, double v3) {
	this->v1= v1;
	this->v2 = v2;
	this->v3 = v3;
}

double Vector::getV1() {
	return this->v1;
}

double Vector::getV2() {
	return this->v2;
}

double Vector::getV3() {
	return this->v3;
}

void Vector::setV1(double v1) {
	this->v1 = v1;
}
void Vector::setV2(double v2) {
	this->v2 = v2;
}

void Vector::setV3(double v3) {
	this->v3 = v3;
}


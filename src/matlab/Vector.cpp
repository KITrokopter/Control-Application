/*
 * Vector.cpp
 *
 *  Created on: 18.01.2014
 *      Author: daniela
 */

#include "Vector.h"
#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <iostream>
#include <sstream>
#include <cstring>
#include <math.h>

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

Vector Vector::add(Vector a) {
    double v1 = this->v1 + a.getV1();
    double v2 = this->v2 + a.getV2();
    double v3 = this->v3 + a.getV3();
    return *(new Vector(v1, v2, v3));
}

Vector Vector::mult(double a) {
    return *(new Vector(this->v1 * a, this->v2 * a, this->v3 * a));
}

int Vector::scalarMult(Vector a) {
    return (this->v1*a.getV1() + this->v2*a.getV2() + this->v3*a.getV3());
}

double Vector::getLength() {
    return sqrt((this->v1*this->v1 + this->v2*this->v2 + this->v3*this->v3));
}

void Vector::putVariable(std::string a, Engine *ep) {
    double datav1[1] = {v1};
    double datav2[1] = {v2};
    double datav3[1] = {v3};
    mxArray *v1 = mxCreateDoubleMatrix(1, 1, mxREAL);
    memcpy((void *)mxGetPr(v1), (void *)datav1, sizeof(datav1));
    std::string out = a + "1";
    engPutVariable(ep, out.c_str(), v1);
    mxArray *v2 = mxCreateDoubleMatrix(1, 1, mxREAL);
    memcpy((void *)mxGetPr(v2), (void *)datav2, sizeof(datav2));
    out = a + "2";
    engPutVariable(ep, out.c_str(), v2);
    mxArray *v3 = mxCreateDoubleMatrix(1, 1, mxREAL);
    memcpy((void *)mxGetPr(v3), (void *)datav3, sizeof(datav3));
    out = a + "3";
    engPutVariable(ep, out.c_str(), v3);
    out = a+" = [" + a + "1, " + a + "2, " + a + "3]";
    engEvalString(ep, out.c_str());
    mxDestroyArray(v1);
    mxDestroyArray(v2);
    mxDestroyArray(v3);
}

Vector Vector::cross(Vector v) {
    Vector *cross = new Vector(v.getV2()*v3 - v.getV3()*v2, v.getV3()*v1 - v.getV1()*v3, v.getV1()*v2 -v.getV2()*v1);
    return *cross;
}

std::string Vector::toString() {
	std::stringstream ss;
	ss << v1 << "/" << v2 << "/" << v3;
	return ss.str();
}
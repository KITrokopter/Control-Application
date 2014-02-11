/*
 * Line.cpp
 *
 *  Created on: 21.01.2014
 *      Author: daniela
 */

#include "Line.h"
#include "Vector.h"

Line::Line(Vector a, Vector u) {
    this->a = a;
    this->u = u;
}

Vector Line::getA() {
	return (this->a);
}

Vector Line::getU() {
	return (this->u);
}

void Line::setA(Vector a) {
	this->a = a;
}

void Line::setU(Vector u) {
	this->u = u;
}





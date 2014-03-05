/*
 * Line.h
 *
 *  Created on: 21.01.2014
 *      Author: daniela
 */

#ifndef LINE_H_
#define LINE_H_
#include "Vector.h"

class Line {
private:/*
	 * a is point on line
	 * u is direction vector of line
	 */ 
	Vector a, u;
public:
	Line();
	Line(Vector a, Vector u);
	Vector getA();
	Vector getU();
	void setA(Vector a);
	void setU(Vector u);
};

#endif /* LINE_H_ */

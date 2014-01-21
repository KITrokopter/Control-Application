/*
 * Vector.h
 *
 *  Created on: 18.01.2014
 *      Author: daniela
 */

#ifndef VECTOR_H_
#define VECTOR_H_



class Vector {
private: double v1, v2, v3;
public:
	Vector(double v1, double v2, double v3);
	Vector();
	double getV1();
	double getV2();
	double getV3();
	void setV1(double v1);
	void setV2(double v2);
	void setV3(double v3);
};

#endif /* VECTOR_H_ */

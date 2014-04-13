/**
 * Line.h
 * Line consisting of one point on the line and one direction vector
 *
 * @author: Daniela Grimm
 **/

#ifndef LINE_H_
#define LINE_H_
#include "Vector.h"

class Line {
private:
    /**
     * a is point on line and u is direction vector of line
     */
	Vector a, u;

public:
    /**
     * empty constructor.
     */
	Line();

    /**
     * constructor.
     * @param a is point on line
     * @param u is direction vector of line
     */
	Line(Vector a, Vector u);

    /**
     * getter.
     * @return point on line
     */
	Vector getA();

    /**
     * getter.
     * @return direction vector of line
     */
	Vector getU();

    /**
     * setter.
     * @param a is point on line
     */
	void setA(Vector a);

    /**
     * setter.
     * @param u is direction vector of line
     */
	void setU(Vector u);
};

#endif /* LINE_H_ */

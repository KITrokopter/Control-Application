/**
 * Matrix.h
 * 3x3 matrix class
 *
 * @author: Daniela Grimm
 **/

#ifndef MATRIX_H_
#define MATRIX_H_

class Matrix {
private:
    /**
     * values of the Matrix, mij is value of i. line and j. column
     */
    double m11, m12, m13, m21, m22, m23, m31, m32, m33;

    /**
     * calculates a determinant of a 2x2 matrix
     * @param a m11
     * @param b m12
     * @param c m21
     * @param d m22
     * @return
     */
    double determinant2x2(double a, double b, double c, double d);

    /**
     * whether Matrix is valid or not
     */
    bool valid;
public:
    /**
     * empty constructor.
     */
    Matrix();

    /**
     * constructor.
     * @param valid whether Matrix is valid or not
     */
    Matrix(bool valid);

    /**
     * constructor.
     * @param m11 first row, first column
     * @param m12 first row, second column
     * @param m13 first row, third column
     * @param m21 second row, first column
     * @param m22 second row, second column
     * @param m23 second row, third column
     * @param m31 third row, first column
     * @param m32 third row, second column
     * @param m33 third row, third column
     */
	Matrix(double m11, double m12, double m13, double m21, double m22, double m23, double m31, double m32, double m33);

    /**
     * getter.
     * @return first row, first column
     */
    double getM11();

    /**
     * getter.
     * @return first row, second column
     */
	double getM12();

    /**
     * getter.
     * @return first row, third column
     */
	double getM13();

    /**
     * getter.
     * @return second row, first column
     */
	double getM21();

    /**
     * getter.
     * @return second row, second column
     */
	double getM22();

    /**
     * getter.
     * @return second row, third column
     */
	double getM23();

    /**
     * getter.
     * @return third row, first column
     */
	double getM31();

    /**
     * getter.
     * @return third row, second column
     */
	double getM32();

    /**
     * getter.
     * @return third row, third column
     */
	double getM33();

    /**
     * print matrix in three lines
     */
	void printMatrix();

    /**
     * matrix multiplication
     * @param A factor
     * @return product
     */
    Matrix multiplicate(Matrix A);

    /**
     * calculates determinant of 3x3 Matrix
     * @return determinant
     */
    double determinant();

    /**
     * calculates inverse of Matrix
     * @return inverse
     */
    Matrix inverse();

    /**
     * getter.
     * @return whether Matrix is valid or not
     */
    bool getValid();

};

#endif /* MATRIX_H_ */

#ifndef PDICONTROL_HPP_
#define PDICONTROL_HPP_

#include "Control.hpp"


class PDIControl : public Control {

public:

	/**
	 *
	 * @param pFactor
	 * @param dFactor
	 * @param offset
	 */
	PDIControl( double pFactor, double dFactor, double offset );

	/**
	 *
	 * @param pFactorPos
	 * @param pFactorNeg
	 * @param dFactor
	 * @param offset
	 */
	PDIControl( double pFactorPos, double pFactorNeg, double dFactor, double offset );

	/**
	 *
	 * @param pFactorPos
	 * @param pFactorNeg
	 * @param dFactor
	 * @param iFactor
	 * @param offset
	 */
	PDIControl( double pFactorPos, double pFactorNeg, double dFactor, double iFactor, double offset );

	/**
	 *
	 * @return
	 */
	double getAmplification();

	/**
	 *
	 * @param errorSignal
	 * @return
	 */
	double getManipulatedVariable( double errorSignal );

	/**
	 *
	 * @param offset
	 */
	void setOffset( double offset );

protected:

	/**
	 *
	 * @param errorSignal
	 */
	void setPAmplification( double errorSignal );
	
private:
	
	double pAmplification;
	double pAmplificationPos;
	double pAmplificationNeg;
	double dAmplification;	
	double iAmplification;
	double offset;
	double distanceOld;
	double integrator;

};

#endif /* PDICONTROL_HPP_ */

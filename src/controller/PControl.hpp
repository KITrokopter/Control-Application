#ifndef PCONTROL_HPP_
#define PCONTROL_HPP_

#include "Control.hpp"


class PControl : public Control {

public:

	/**
	 *
	 * @param pFactor
	 * @param offset
	 */
	PControl( double pFactor, double offset );

	/**
	 *
	 * @param pFactorPos
	 * @param pFactorNeg
	 * @param offset
	 */
	PControl( double pFactorPos, double pFactorNeg, double offset );


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
	double offset;
};

#endif /* PCONTROL_HPP_ */

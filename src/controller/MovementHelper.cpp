/*
 * MovementHelper.cpp
 *
 *  Created on: 30.03.2014
 *      Author: dwx
 */

#include "MovementHelper.hpp"

MovementHelper::MovementHelper() {


}

Position6DOF MovementHelper::prepareForRP( double rotation, Position6DOF pos, Position6DOF target )
{
	/* values of the Matrix, mij is value of i. line and j. column  */
	double m11 = cos( rotation );
	double m12 = -sin( rotation );
	double m21 = sin( rotation );
	double m22 = cos( rotation );
	double *current = pos.getPosition();
	double newPosition[3];
	newPosition[0] = m11 * current[0] + m12 * current[1];
	newPosition[1] = m21 * current[0] + m22 * current[1];
	newPosition[2] = current[2];

	pos.setPosition( newPosition );
	return pos;
}

//double getHeightDiff( std::list<Position6DOF> listPositions )
//{
//	double heightLatest = listPositions.front().getThrust();
//	double heightPast;
//
//	std::list<Position6DOF>::iterator it;
//	int counter = 0;
//	for( it=listPositions.begin(); (it!=listPositions.end()) && (counter<2); ++it )
//	{
//		if( counter == 1 )
//		{
//			positionPast.setOrientation( it->getOrientation() );
//			positionPast.setPosition( it->getPosition() );
//			positionPast.setTimestamp( it->getTimestamp() );
//		}
//		counter++;
//	}
////	double heightLatest = listPositions.front().getThrust();
////	double heightPast;
////
////	std::list<Position6DOF>::iterator it;
////	int counter = 0;
////	for( it=listPositions.begin(); (it!=listPositions.end()) && (counter<2); ++it )
////	{
////		if( counter == 1 )
////		{
////			positionPast.setOrientation( it->getOrientation() );
////			positionPast.setPosition( it->getPosition() );
////			positionPast.setTimestamp( it->getTimestamp() );
////		}
////		counter++;
////	}
//}
//
//double getHeightDiffReverse( std::list<Position6DOF> listPositions )
//{
//	double heightLatest = listPositions;
//	Position6DOF positionNow = listPositions.back();	// positionPast is older than positionNow
//	Position6DOF positionPast;
//
//	std::list<Position6DOF>::reverse_iterator rit;
//	int counter = 0;
//	for( rit=listPositions.rbegin(); (rit!=listPositions.rend()) && (counter<2); ++rit )
//	{
//		if( counter == 1 )
//		{
//			positionPast.setOrientation( rit->getOrientation() );
//			positionPast.setPosition( rit->getPosition() );
//			positionPast.setTimestamp( rit->getTimestamp() );
//		}
//		counter++;
//	}
//}

#include "ChessboardData.hpp"

/**
 * Creates the container storing the given parameters.
 * 
 * @param numberCornersX The amount of corners on the x-axis of the chessboard.
 * @param numberCornersY The amount of corners on the y-axis of the chessboard.
 * @param chessFieldWidth The width of one field of the chessboard.
 * @param chessFieldHeight The height of one field of the chessboard.
 */
ChessboardData::ChessboardData(int numberCornersX, int numberCornersY, double chessFieldWidth, double chessFieldHeight)
{
	this->numberCornersX = numberCornersX;
	this->numberCornersY = numberCornersY;
	this->chessFieldWidth = chessFieldWidth;
	this->chessFieldHeight = chessFieldHeight;
}

/**
 * Returns the width of one field of the chessboard.
 * 
 * @return The width of one field of the chessboard.
 */
double ChessboardData::getChessFieldWidth() {
	return chessFieldWidth;
}

/**
 * Returns the height of one field of the chessboard.
 * 
 * @return The height of one field of the chessboard.
 */
double ChessboardData::getChessFieldHeight() {
	return chessFieldHeight;
}

/**
 * Returns the amount of corners on the x-axis of the chessboard.
 * 
 * @return The amount of corners on the x-axis of the chessboard.
 */
int ChessboardData::getNumberCornersX() {
	return numberCornersX;
}

/**
 * Returns the amount of corners on the y-axis of the chessboard.
 * 
 * @return The amount of corners on the y-axis of the chessboard.
 */
int ChessboardData::getNumberCornersY() {
	return numberCornersY;
}
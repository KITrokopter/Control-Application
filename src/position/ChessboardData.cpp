#include "ChessboardData.hpp"

ChessboardData::ChessboardData(int numberCornersX, int numberCornersY, double chessFieldWidth, double chessFieldHeight)
{
	this->numberCornersX = numberCornersX;
	this->numberCornersY = numberCornersY;
	this->chessFieldWidth = chessFieldWidth;
	this->chessFieldHeight = chessFieldHeight;
}

double ChessboardData::getChessFieldWidth() {
	return chessFieldWidth;
}

double ChessboardData::getChessFieldHeight() {
	return chessFieldHeight;
}

int ChessboardData::getNumberCornersX() {
	return numberCornersX;
}

int ChessboardData::getNumberCornersY() {
	return numberCornersY;
}
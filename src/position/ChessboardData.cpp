#include "ChessboardData.hpp"

ChessboardData::ChessboardData(double chessboardWidthPixels, double chessboardHeightPixels, double chessboardWidth,
               double chessboardHeight, double imageWidthPixel, double imageHeightPixel, double horizonalBeta,
               double verticalBeta, double chessboardHorizontalOffsetPixels, double chessboardVerticalOffsetPixels)
{
    this->chessboardWidthPixels = chessboardWidthPixels;
    this->chessboardHeightPixels = chessboardHeightPixels;
    this->chessboardWidth = chessboardWidth;
    this->chessboardHeight = chessboardHeight;
    this->imageWidthPixel = imageWidthPixel;
    this->imageHeightPixel = imageHeightPixel;
    this->horizonalBeta = horizonalBeta;
    this->verticalBeta = verticalBeta;
    this->chessboardHorizontalOffsetPixels = chessboardHorizontalOffsetPixels;
    this->chessboardVerticalOffsetPixels = chessboardVerticalOffsetPixels;
}

ChessboardData::ChessboardData(double chessboardWidthPixels, double chessboardHeightPixels, double chessboardWidth,
			   double chessboardHeight, double imageWidthPixel, double imageHeightPixel, double horizonalBeta,
               double verticalBeta, double chessboardHorizontalOffsetPixels, double chessboardVerticalOffsetPixels,
               int numberFieldsX, int numberFieldsY, double chessFieldWidth, double chessFieldHeight)
{
	this->chessboardWidthPixels = chessboardWidthPixels;
	this->chessboardHeightPixels = chessboardHeightPixels;
	this->chessboardWidth = chessboardWidth;
	this->chessboardHeight = chessboardHeight;
	this->imageWidthPixel = imageWidthPixel;
	this->imageHeightPixel = imageHeightPixel;
	this->horizonalBeta = horizonalBeta;
	this->verticalBeta = verticalBeta;
	this->chessboardHorizontalOffsetPixels = chessboardHorizontalOffsetPixels;
	this->chessboardVerticalOffsetPixels = chessboardVerticalOffsetPixels;
    this->numberFieldsX = numberFieldsX;
    this->numberFieldsY = numberFieldsY;
    this->chessFieldWidth = chessFieldWidth;
    this->chessFieldHeight = chessFieldHeight;
}

ChessboardData::ChessboardData(double chessboardWidthPixels, double chessboardHeightPixels, double imageWidthPixel,
			   double imageHeightPixel, double horizonalBeta, double verticalBeta,
			   double chessboardHorizontalOffsetPixels, double chessboardVerticalOffsetPixels)
{
	this->chessboardWidthPixels = chessboardWidthPixels;
	this->chessboardHeightPixels = chessboardHeightPixels;
	this->imageWidthPixel = imageWidthPixel;
	this->imageHeightPixel = imageHeightPixel;
	this->horizonalBeta = horizonalBeta;
	this->verticalBeta = verticalBeta;
	this->chessboardHorizontalOffsetPixels = chessboardHorizontalOffsetPixels;
	this->chessboardVerticalOffsetPixels = chessboardVerticalOffsetPixels;
}

void ChessboardData::setChessboardWidth(double chessboardWidth)
{
	this->chessboardWidth = chessboardWidth;
}

void ChessboardData::setChessboardHeight(double chessboardHeight)
{
	this->chessboardHeight = chessboardHeight;
}

double ChessboardData::getChessboardWidthPixels()
{
	return chessboardWidthPixels;
}

double ChessboardData::getChessboardHeightPixels()
{
	return chessboardHeightPixels;
}

double ChessboardData::getChessboardWidth()
{
	return chessboardWidth;
}

double ChessboardData::getChessboardHeight()
{
	return chessboardHeight;
}

double ChessboardData::getChessFieldWidth() {
    return chessFieldWidth;
}

double ChessboardData::getChessFieldHeight() {
    return chessFieldHeight;
}

double ChessboardData::getImageWidthPixel()
{
	return imageWidthPixel;
}

double ChessboardData::getImageHeightPixel()
{
	return imageHeightPixel;
}

double ChessboardData::getHorizonalBeta()
{
	return horizonalBeta;
}

double ChessboardData::getVerticalBeta()
{
	return verticalBeta;
}

int ChessboardData::getNumberFieldsX() {
    return numberFieldsX;
}

int ChessboardData::getNumberFieldsY() {
    return numberFieldsY;
}

double ChessboardData::getChessboardHorizontalOffsetPixels()
{
	return chessboardHorizontalOffsetPixels;
}

double ChessboardData::getChessboardVerticalOffsetPixels()
{
	return chessboardVerticalOffsetPixels;
}

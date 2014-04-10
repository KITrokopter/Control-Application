#pragma once

/**
 * A container for the geometric attributes of the chessboard.
 * 
 * @author Sebastian Schmidt
 */
class ChessboardData {
private:
	/// The vertical number of fields
	int numberCornersX;
	/// The horizontal number of fields
	int numberCornersY;
	
	/// The real width of one field on the chessboard
	double chessFieldWidth;
	/// The real height of one field on the chessboard
	double chessFieldHeight;
	
public:
	ChessboardData(int numberCornersX, int numberCornersY, double chessFieldWidth, double chessFieldHeight);

	int getNumberCornersX();
	int getNumberCornersY();
	
	/// The real widht of one field on the chessboard
	double getChessFieldWidth();
	/// The real height of one field on the chessboard
	double getChessFieldHeight();
};

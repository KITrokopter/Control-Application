#pragma once

class ChessboardData {
private:
	/// The width of the chessboard as seen on the image in pixels. (ls')
	double chessboardWidthPixels;
	/// The height of the chessboard as seen on the image in pixels.
	double chessboardHeightPixels;
	/// The real width of the chessboard in mm. (ls)
	double chessboardWidth;
	/// The real height of the chessboard in mm.
	double chessboardHeight;
	
    /// The real width of one field on the chessboard
    double chessFieldWidth;
    /// The real height of one field on the chessboard
    double chessFieldHeight;

	/// The width of the complete image in pixels. (2b')
	double imageWidthPixel;
	/// The height of the complete image in pixels.
	double imageHeightPixel;
	
	/// The horizontal opening angle of the camera in degrees. (beta)
	double horizonalBeta;
	/// The vertical opening angle of the camera in degrees.
	double verticalBeta;
	
    /// The vertical number of fields
    int numberFieldsX;
    /// The horizontal number of fields
    int numberFieldsY;

	/// The horizontal offset of the chessboard compared to the center in pixels. (g')
	/// Note that a positive value means the chessboard is moved to the right.
	double chessboardHorizontalOffsetPixels;
	/// The vertical offset of the chessboard compared to the center in pixels.
	/// Note that a positive value means the chessboard is moved to the top.
	double chessboardVerticalOffsetPixels;
	
public:
    ChessboardData(double chessboardWidthPixels, double chessboardHeightPixels, double chessboardWidth,
                   double chessboardHeight, double imageWidthPixel, double imageHeightPixel, double horizonalBeta,
                   double verticalBeta, double chessboardHorizontalOffsetPixels, double chessboardVerticalOffsetPixels);

    ChessboardData(double chessboardWidthPixels, double chessboardHeightPixels, double chessboardWidth,
                   double chessboardHeight, double imageWidthPixel, double imageHeightPixel, double horizonalBeta,
                   double verticalBeta, double chessboardHorizontalOffsetPixels, double chessboardVerticalOffsetPixels,
                   int numberFieldsX, int numberFieldsY, double chessFieldWidth, double chessFieldHeight);

	ChessboardData(double chessboardWidthPixels, double chessboardHeightPixels, double imageWidthPixel,
				   double imageHeightPixel, double horizonalBeta, double verticalBeta,
				   double chessboardHorizontalOffsetPixels, double chessboardVerticalOffsetPixels);
	
	void setChessboardWidth(double chessboardWidth);
	void setChessboardHeight(double chessboardHeight);
	
	/// The width of the chessboard as seen on the image in pixels. (ls')
	double getChessboardWidthPixels();
	/// The height of the chessboard as seen on the image in pixels.
	double getChessboardHeightPixels();
	/// The real width of the chessboard in mm. (ls)
	double getChessboardWidth();
	/// The real height of the chessboard in mm.
	double getChessboardHeight();
    /// The real widht of one field on the chessboard
    double getChessFieldWidth();
    /// The real height of one field on the chessboard
    double getChessFieldHeight();

	/// The width of the complete image in pixels. (2b')
	double getImageWidthPixel();
	/// The height of the complete image in pixels.
    double getImageHeightPixel();

	/// The horizontal opening angle of the camera in degrees. (beta)
	double getHorizonalBeta();
	/// The vertical opening angle of the camera in degrees.
	double getVerticalBeta();

    int getNumberFieldsX();
    int getNumberFieldsY();

	/// The horizontal offset of the chessboard compared to the center in pixels. (g')
	/// Note that a positive value means the chessboard is moved to the right.
	double getChessboardHorizontalOffsetPixels();
	/// The vertical offset of the chessboard compared to the center in pixels.
	/// Note that a positive value means the chessboard is moved to the top.
	double getChessboardVerticalOffsetPixels();
};

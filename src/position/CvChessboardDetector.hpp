#pragma once

#include "ChessboardData.hpp"
#include <opencv2/core/core.hpp>

class CvChessboardDetector {
private:
	cv::Size lastSize;
	std::vector<cv::Point3f>* objectPoints;
	
	std::vector<cv::Point3f>* createObjectPoints(cv::Size size, cv::Size realSize);
	cv::Point2f checkBigger(cv::Point2f current, cv::Point2f toCheck, int x, int y);
	
public:
	CvChessboardDetector();
	~CvChessboardDetector();
	
	/**
	 * Calculate the chessboard data from the given image, if a chessboard was found.
	 * 
	 * @param image The image.
	 * @param boardSize The amount of corners the board has.
	 * @param realSize The real size of the chessboard. Defined by the outermost corners.
	 * @return The data extracted from the image, or 0 if no chessboard was found.
	 */
	ChessboardData* detectChessboard(cv::Mat* image, cv::Size boardSize, cv::Size realSize);
};
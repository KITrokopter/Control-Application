#include "CvChessboardDetector.hpp"

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>

CvChessboardDetector::CvChessboardDetector() : lastSize(0, 0), objectPoints(0) {}

CvChessboardDetector::~CvChessboardDetector()
{
	if (objectPoints != 0) {
		delete objectPoints;
	}
}

std::vector<cv::Point3f>* CvChessboardDetector::createObjectPoints(cv::Size size, cv::Size realSize)
{
	if (objectPoints != 0) {
		if (size.width == lastSize.width && size.height == lastSize.height) {
			return objectPoints;
		} else {
			delete objectPoints;
		}
	}
	
	objectPoints = new std::vector<cv::Point3f>();
	
	for (int i = 0; i < size.width; i++) {
		for (int j = 0; j < size.height; j++) {
			cv::Point3f point(i * realSize.width, j * realSize.height, 0);
			objectPoints->push_back(point);
		}
	}
	
	return objectPoints;
}

cv::Point2f CvChessboardDetector::checkBigger(cv::Point2f current, cv::Point2f toCheck, int x, int y)
{
	if (toCheck.x * x + toCheck.y * y > current.x * x + current.y * y) {
		return toCheck;
	} else {
		return current;
	}
}

ChessboardData* CvChessboardDetector::detectChessboard(cv::Mat* image, cv::Size boardSize, cv::Size realSize)
{
	std::vector<cv::Point3f> *objectPoints = createObjectPoints(boardSize, cv::Size(realSize.width / (boardSize.width - 1),
																				realSize.height / (boardSize.height - 1)));
	
	// Find chessboard corners
	std::vector<cv::Point2f> corners;
	bool foundAllCorners = cv::findChessboardCorners(*image, boardSize, corners,
			CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);
	
	if (!foundAllCorners) {
		return 0;
	}
	
	// Sub pixel accuracy
	cv::Mat greyImage(image->size(), CV_8UC1);
	cv::cvtColor(*image, greyImage, CV_RGB2GRAY);
	
	// TODO: Good termination values? Good cv::Size values?
	cv::cornerSubPix(greyImage, corners, cv::Size(5, 5), cv::Size(-1, -1),
					 cv::TermCriteria(cv::TermCriteria::MAX_ITER | cv::TermCriteria::EPS, 30, 0.1));
	
	cv::Point2f topLeft = corners[0];
	cv::Point2f topRight = corners[0];
	cv::Point2f bottomLeft = corners[0];
	cv::Point2f bottomRight = corners[0];
	cv::Point2f center = corners[0];
	
	std::vector<cv::Point2f>::iterator it = corners.begin();
	it++;
	
	for (; it != corners.end(); it++) {
		topLeft = checkBigger(topLeft, *it, -1, -1);
		topRight = checkBigger(topRight, *it, 1, -1);
		bottomLeft = checkBigger(bottomLeft, *it, -1, 1);
		bottomRight = checkBigger(bottomRight, *it, 1, 1);
		
		center.x += it->x;
		center.y += it->y;
	}
	
	center.x /= corners.size();
	center.y /= corners.size();
	
	double chessboardWidthPixels = (topRight.x - topLeft.x + bottomRight.x - bottomLeft.x) / 2;
	double chessboardHeightPixels = -(topRight.y - bottomRight.y + topLeft.y - bottomLeft.y) / 2;
	
	double chessboardHorizontalOffsetPixels = center.x - 320;
	double chessboardVerticalOffsetPixels = 240 - center.y;
	
	return new ChessboardData(chessboardWidthPixels, chessboardHeightPixels, realSize.width, realSize.height, 640, 480,
							  57.0 / 2, 43.0 / 2, chessboardHorizontalOffsetPixels, chessboardVerticalOffsetPixels);
}
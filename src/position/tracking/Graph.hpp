#pragma once

#include <opencv2/core/core.hpp>

#include <string>
#include <map>

class Graph
{
public:
	
// 	Graph(int maxValue, std::string windowName, std::map<int, cv::Scalar> colors);
	Graph(int maxValue, std::string windowName);
	~Graph();
	
	void nextPoint(double value, int id);
	void setColors(std::map<int, cv::Scalar> colors);
	
private:
	
	void drawMetadata();
	uchar* getPixel(cv::Mat mat, int x, int y);
	
	cv::Mat image;
	std::string windowName;
	std::map<int, cv::Scalar> colors;
	int imageWidth, imageHeight;
	int graphAreaWidth, graphAreaHeight;
	int graphAreaHorizontalOffset, graphAreaVerticalOffset;
	int maxValue;
	int currentPosition;
	int *graph;
	bool *graphInitialized;
	
};
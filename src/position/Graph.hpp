#pragma once

#include <opencv2/core/core.hpp>

#include <string>

class Graph
{
public:
	
	Graph(int maxValue, std::string windowName);
	~Graph();
	
	void nextPoint(double value);
	
private:
	
	void drawMetadata();
	uchar* getPixel(cv::Mat mat, int x, int y);
	
	cv::Mat image;
	std::string windowName;
	int imageWidth, imageHeight;
	int graphAreaWidth, graphAreaHeight;
	int graphAreaHorizontalOffset, graphAreaVerticalOffset;
	int maxValue;
	int currentPosition;
	int *graph;
	bool *graphInitialized;
	
};
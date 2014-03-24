#include "Graph.hpp"

#include <opencv2/highgui/highgui.hpp>
#include <ros/console.h>

#include <sstream>

Graph::Graph(int maxValue, std::string windowName)
{
	this->maxValue = maxValue;
	this->windowName = windowName;
	this->colors = colors;
	
	imageWidth = 640;
	imageHeight = 480;
	graphAreaWidth = 600;
	graphAreaHeight = 440;
	graphAreaHorizontalOffset = 40;
	graphAreaVerticalOffset = 30;
	currentPosition = 0;
	graph = new int[graphAreaWidth];
	graphInitialized = new bool[graphAreaWidth];
	
	for (int i = 0; i < graphAreaWidth; i++) {
		graphInitialized[i] = false;
	}
	
	image = cv::Mat::zeros(cv::Size(imageWidth, imageHeight), CV_8UC3);
	drawMetadata();
	
	cv::startWindowThread();
	cv::namedWindow(windowName);
}

void Graph::setColors(std::map<int, cv::Scalar> colors)
{
	this->colors = colors;
}

void Graph::drawMetadata()
{
	std::stringstream ssMaxValue;
	ssMaxValue << maxValue;
	std::stringstream ssZero;
	ssZero << "0";
	
	cv::putText(image, ssMaxValue.str(), cv::Point(10, imageHeight - graphAreaHeight - graphAreaVerticalOffset - 30),
				cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(255, 255, 255));
	cv::putText(image, ssZero.str(), cv::Point(10, imageHeight - graphAreaVerticalOffset),
				cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(255, 255, 255));
	
	cv::rectangle(image, cv::Point(graphAreaHorizontalOffset - 1, imageHeight - graphAreaVerticalOffset - graphAreaHeight - 1),
				  cv::Point(graphAreaHorizontalOffset + graphAreaWidth + 1, imageHeight - graphAreaVerticalOffset + 1), cv::Scalar(150, 150, 150));
}

uchar* Graph::getPixel(cv::Mat mat, int x, int y)
{
	return mat.data + mat.step[0] * y + mat.step[1] * x;
}

void Graph::nextPoint(double value, int id)
{
	if (!colors.count(id)) {
		ROS_ERROR("id %d has no color!", id);
		return;
	}
	
	int intValue = (int) (value / maxValue * graphAreaHeight);
	
	if (graphInitialized[currentPosition]) {
		uchar* pixel = getPixel(image, graphAreaHorizontalOffset + currentPosition, imageHeight - graphAreaVerticalOffset - graph[currentPosition]);
		pixel[0] = 0;
		pixel[1] = 0;
		pixel[2] = 0;
	} else {
		graphInitialized[currentPosition] = true;
	}
	
	cv::Scalar color = colors[id];
	
	if (intValue > graphAreaHeight) {
		color = cv::Scalar(0, 0, 255);
		intValue = graphAreaHeight;
	}
	
	graph[currentPosition] = intValue;
	
	uchar* newPixel = getPixel(image, graphAreaHorizontalOffset + currentPosition, imageHeight - graphAreaVerticalOffset - intValue);
	newPixel[0] = color[0];
	newPixel[1] = color[1];
	newPixel[2] = color[2];
	
	currentPosition = (currentPosition + 1) % graphAreaWidth;
	
	cv::imshow(windowName, image);
}

Graph::~Graph()
{
	cv::destroyWindow(windowName);
}
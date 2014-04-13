#include "Graph.hpp"

#include <opencv2/highgui/highgui.hpp>
#include <ros/console.h>

#include <sstream>

/**
 * Creates a new empty graph window.
 *
 * @param maxValue The maximum value of the graph scale. The minimum
 * value is always 0.
 * @param windowName The title of the window the graph will be
 * displayed in.
 */
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
	cv::imshow(windowName, image);
}

/**
 * Sets the id -> color map for the graph.
 *
 * @param colors The color map.
 */
void Graph::setColors(std::map<int, cv::Scalar> colors)
{
	this->colors = colors;
}

/**
 * Draws the surroundings of the graph.
 */
void Graph::drawMetadata()
{
	std::stringstream ssMaxValue;
	ssMaxValue << maxValue;
	std::stringstream ssZero;
	ssZero << "0";

	cv::putText(image, ssMaxValue.str(), cv::Point(10, imageHeight - graphAreaHeight - graphAreaVerticalOffset),
	            cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(255, 255, 255));
	cv::putText(image, ssZero.str(), cv::Point(10, imageHeight - graphAreaVerticalOffset),
	            cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(255, 255, 255));

	cv::rectangle(image,
	              cv::Point(graphAreaHorizontalOffset - 1, imageHeight - graphAreaVerticalOffset - graphAreaHeight - 1),
	              cv::Point(graphAreaHorizontalOffset + graphAreaWidth + 1,
	                        imageHeight - graphAreaVerticalOffset + 1), cv::Scalar(150, 150, 150));
}

/**
 * Returns a pointer to the pixel at the gixen coordinates.
 *
 * @param mat The image that contains the pixel.
 * @param x The x coordinate of the pixel.
 * @param y The y coordinate of the pixel.
 */
uchar* Graph::getPixel(cv::Mat &mat, int x, int y)
{
	return mat.data + mat.step[0] * y + mat.step[1] * x;
}

/**
 * Draws the given point to the graph.
 *
 * @param value The y value.
 * @param id The id to select the color using the color map.
 */
void Graph::nextPoint(double value, int id)
{
	if (!colors.count(id)) {
		ROS_ERROR("id %d has no color!", id);
		return;
	}

	int intValue = (int) (value / maxValue * graphAreaHeight);

	if (graphInitialized[currentPosition]) {
		uchar *pixel = getPixel(image, graphAreaHorizontalOffset + currentPosition,
		                        imageHeight - graphAreaVerticalOffset - graph[currentPosition]);
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

	uchar *newPixel = getPixel(image, graphAreaHorizontalOffset + currentPosition,
	                           imageHeight - graphAreaVerticalOffset - intValue);
	newPixel[0] = color[0];
	newPixel[1] = color[1];
	newPixel[2] = color[2];

	currentPosition = (currentPosition + 1) % graphAreaWidth;

	cv::imshow(windowName, image);
}

/**
 * Destroys the graph and the graph window.
 */
Graph::~Graph()
{
	cv::destroyWindow(windowName);
}
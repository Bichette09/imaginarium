#pragma once

// opencv
#include "opencv2/opencv.hpp"

// std
#include <map>
#include <chrono>

class FrameInterface
{
public:
	FrameInterface();
	virtual ~FrameInterface();
	
	virtual cv::Mat & editY() = 0;
	virtual cv::Mat & editU() = 0;
	virtual cv::Mat & editV() = 0;
	virtual void setGrabTimestamp() = 0;
	
	void ensureSizeAndType(int pWidth, int pHeight);
	
private:

};

class FrameProvider
{
public:
	virtual ~FrameProvider(){};
	
	virtual int getFrameWidth() const = 0;
	virtual int getFrameHeight() const = 0;
	
	virtual bool getNextFrame(FrameInterface & pFrame) = 0;
private:
};
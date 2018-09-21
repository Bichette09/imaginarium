#pragma once

// opencv
#include "opencv2/opencv.hpp"

// std
#include <map>
#include <chrono>

#include "light_and_line_frame.h"

class LineDetector
{
public:
	typedef std::vector<cv::Rect> tRects;
	
	LineDetector(int pWidthResolution);
	virtual ~LineDetector();
	
	typedef int32_t tTs;
	
	
	void addNewFrame(const LightAndLineFrame & pFrame);
	void clearDetector();
	bool detectLineSequence();
	
	void createDebugImg(cv::Mat & pTarget,tTs pTimeWindowSec);
	
private:
	
	typedef std::list<tTs> tTsQueue;
	typedef std::vector<tTsQueue> tDetectionCells;
	
	const std::chrono::time_point<std::chrono::system_clock>	mCreationTime;
	tTs			mCurrentTs;
	
	tDetectionCells	mDetectionCells[LightAndLineFrame::LT_Count];
	
	const int	mWidth;
	cv::Rect	mLineSearchRoi;
};
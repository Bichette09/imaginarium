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
	
	
	void addNewFrame(const LightAndLineFrame & pFrame, tTs pTimeWindowSec);
	void clearDetector();
	bool detectLineSequence();
	
	void createDebugImg(cv::Mat & pTarget,tTs pTimeWindowSec);
	
	bool detectLine(LightAndLineFrame::LinesType pLineType);
	
private:

	void addLineColorAreas(const LightAndLineFrame & pFrame, LightAndLineFrame::ColorAreas pColorArea, LightAndLineFrame::LinesType pLineType, tTs pTimeWindowSec);
	
	typedef std::list<tTs> tTsQueue;
	struct DetectionCell
	{
		DetectionCell();
		void clearDetectionCell();
		
		tTsQueue	mTsQueue;
		bool		mNoDataInLastFrame;
	};
	typedef std::vector<DetectionCell> tDetectionCells;
	
	const std::chrono::time_point<std::chrono::system_clock>	mCreationTime;
	tTs			mCurrentTs;
	
	tDetectionCells	mDetectionCells[LightAndLineFrame::LT_Count];
	
	const int	mHeight;
	const int	mWidth;
	
	cv::Rect	mLineSearchRoi;
};
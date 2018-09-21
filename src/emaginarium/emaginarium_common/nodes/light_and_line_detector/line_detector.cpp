#include "line_detector.h"

// std
#include <algorithm>

// ros
#include "ros/ros.h"

LineDetector::LineDetector(int pWidthResolution)
	: mWidth(pWidthResolution)
	, mCreationTime(std::chrono::system_clock::now())
{
	clearDetector();
}

LineDetector::~LineDetector()
{
}

void LineDetector::addNewFrame(const LightAndLineFrame & pFrame)
{
	if(mLineSearchRoi != pFrame.getLineSearchArea())
	{
		clearDetector();
	}
	mLineSearchRoi = pFrame.getLightSearchArea();
	
	
	const std::chrono::time_point<std::chrono::system_clock> lNow(std::chrono::system_clock::now());
	mCurrentTs = std::chrono::duration_cast<std::chrono::milliseconds>(lNow-mCreationTime).count();
	
}


void LineDetector::clearDetector()
{
	for(int i = 0 ; i < LightAndLineFrame::LT_Count ; ++i)
	{
		mDetectionCells[i].clear();
		mDetectionCells[i].resize(mWidth);
	}
	mCurrentTs = 0;
	mLineSearchRoi = cv::Rect();
}

void LineDetector::createDebugImg(cv::Mat & pTarget,tTs pTimeWindowSec)
{
	const tTs lMinTsValue = mCurrentTs - pTimeWindowSec*1000;
	
	pTarget = cv::Mat(1,mWidth,CV_8UC3,cv::Scalar(0,0,0));
	
	
}



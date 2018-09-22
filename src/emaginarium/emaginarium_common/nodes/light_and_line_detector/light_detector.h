#pragma once

// opencv
#include "opencv2/opencv.hpp"

// std
#include <map>
#include <chrono>

#include "light_and_line_frame.h"

class LightDetector
{
public:
	typedef std::vector<cv::Rect> tRects;
	
	LightDetector(int pWidthResolution, int pHeightResolution);
	virtual ~LightDetector();
	
	typedef int32_t tTs;
	
	
	void addNewFrame(const LightAndLineFrame & pFrame);
	void clearDetector();
	bool detectLightSequence(tTs pTimeWindowMSec);
	
	void createDebugImg(cv::Mat & pTarget,tTs pTimeWindowMSec);
	
private:
	struct DetectionCell
	{
		DetectionCell();
	
		tTs	mTs[LightAndLineFrame::LC_Count];
	};
	
	void addDetectedAreas(const tRects & pAreas, LightAndLineFrame::ColorAreas pColor);
	
	typedef std::vector<DetectionCell> tCellVector;
	typedef std::vector< tCellVector > tCellMatrix;
	
	const std::chrono::time_point<std::chrono::system_clock>	mCreationTime;
	tTs			mCurrentTs;
	const int	mHeight;
	const int	mWidth;
	tCellMatrix	mCellMatrix;
	cv::Rect	mLightSearchRoi;
};
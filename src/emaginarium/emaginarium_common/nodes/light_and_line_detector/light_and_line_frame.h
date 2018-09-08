#pragma once

// opencv
#include "opencv2/opencv.hpp"

// std
#include <map>
#include <chrono>

#include "image_common/frame_interface.h"

class LightAndLineFrame : public FrameInterface
{
public:
	
	
	
	LightAndLineFrame();
	virtual ~LightAndLineFrame();
	
	enum Layer
	{
		Y = 0,
		U,
		V,
		Debug,
		Debug2,
		LightStatus,
		
		LayerCount
	};
	typedef std::map<Layer,cv::Mat> tLayers;
	enum TimeStampFence
	{
		F_GrabDone = 0,
		F_LightThresholdingStart,
		F_LightThresholdingDone,
		F_LightAnalyzeStart,
		F_LightAnalyzeDone
	};
	typedef std::chrono::time_point<std::chrono::system_clock> tTimestamp;
	typedef std::map<TimeStampFence,tTimestamp> tTimestamps;
	
	enum LightColor
	{
		LC_Blue = 0,
		LC_Yellow,
		LC_Red,
		LC_Count
	};
	typedef std::vector<cv::Rect> tRects;
	tRects & operator[](LightColor pColor);
	const tRects & operator[](LightColor pColor) const;
	cv::Rect & editLightSearchArea() ;
	const cv::Rect & getLightSearchArea() const;
	
	void swap(LightAndLineFrame & pOther);
	
	cv::Mat & operator[](Layer pLayer);
	const cv::Mat & operator[](Layer pLayer) const;
	
	void setTimestamp(TimeStampFence pFence);
	tTimestamp operator[](TimeStampFence pFence);
	
	virtual cv::Mat & editY();
	virtual cv::Mat & editU();
	virtual cv::Mat & editV();
	virtual void setGrabTimestamp();
private:
	
	tLayers		mLayers;
	tTimestamps	mTimestamps;
	tRects		mLightAreas[LC_Count];
	cv::Rect	mLightSearchArea;
};
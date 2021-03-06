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
		CannyEdges,
		Overlay,
		Debug,
		Debug2,
		LightStatus,
		LineStatus,
		
		LayerCount
	};
	typedef std::map<Layer,cv::Mat> tLayers;
	enum TimeStampFence
	{
		F_GrabDone = 0,
		F_LightThresholdingStart,
		F_LightThresholdingDone,
		F_LightAnalyzeStart,
		F_LightAnalyzeDone,
		F_LineCanyFilterStart,
		F_LineCanyFilterDone,
		F_LineHoughFilterStart,
		F_LineHoughFilterDone
	};
	typedef std::chrono::time_point<std::chrono::system_clock> tTimestamp;
	typedef std::map<TimeStampFence,tTimestamp> tTimestamps;
	
	enum ColorAreas
	{
		LC_Blue = 0,
		LC_Yellow,
		LC_Red,
		
		LC_LineGreen,
		LC_LineRed,
		
		LC_Count
	};
	typedef std::vector<cv::Rect> tRects;
	tRects & operator[](ColorAreas pType);
	const tRects & operator[](ColorAreas pType) const;
	
	
	cv::Rect & editLightSearchArea() ;
	const cv::Rect & getLightSearchArea() const;
	
	enum LinesType
	{
		LT_Red,
		LT_Green,
		
		LT_Count
	};
	
	typedef std::vector<cv::Vec4i> tLines;
	tLines & operator[](LinesType pType);
	const tLines & operator[](LinesType pType) const;
		
	cv::Rect & editLineSearchArea() ;
	const cv::Rect & getLineSearchArea() const;
	
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
	tRects		mColorAreas[LC_Count];
	tLines		mLines[LT_Count];
	cv::Rect	mLightSearchArea;
	cv::Rect	mLineSearchArea;
	
};
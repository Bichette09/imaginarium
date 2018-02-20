#pragma once

// opencv
#include "opencv2/opencv.hpp"

// std
#include <map>
#include <chrono>

class GrabbedFrame
{
public:
	
	GrabbedFrame();
	virtual ~GrabbedFrame();
	
	enum Layer
	{
		Y,
		U,
		V,
		BackgroundMask
	};
	typedef std::map<Layer,cv::Mat> tLayers;
	enum TimeStampFence
	{
		F_GrabDone = 0,
		F_FilterStart,
		F_FilterDone,
		F_AreaExtractionStart,
		F_AreaExtractionDone
	};
	typedef std::chrono::time_point<std::chrono::system_clock> tTimestamp;
	typedef std::map<TimeStampFence,tTimestamp> tTimestamps;
	
	void swap(GrabbedFrame & pOther);
	
	cv::Mat & operator[](Layer pLayer);

	void setTimestamp(TimeStampFence pFence);
	tTimestamp operator[](TimeStampFence pFence);
	
	static void ComputeLatencyAndFps(const tTimestamp & pPreviousTs, const tTimestamp & pNewTs, float & pLatency, float & pFps);

private:
	
	tLayers		mLayers;
	tTimestamps	mTimestamps;
};
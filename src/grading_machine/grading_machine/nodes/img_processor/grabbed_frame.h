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
	typedef std::chrono::time_point<std::chrono::system_clock> tTimestamp;
	
	void swap(GrabbedFrame & pOther);
	
	cv::Mat & operator[](Layer pLayer);
	void setTimestamp();
	tTimestamp getTimestamp() const;
	static void ComputeLatencyAndFps(const tTimestamp & pPreviousTs, const tTimestamp & pNewTs, float & pLatency, float & pFps);

private:
	
	tLayers		mLayers;
	tTimestamp	mGrabTs;
};
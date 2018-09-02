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
		Y,
		U,
		V
	};
	typedef std::map<Layer,cv::Mat> tLayers;
	enum TimeStampFence
	{
		F_GrabDone = 0,
		F_ThresholdingStart,
		F_ThresholdingDone
	};
	typedef std::chrono::time_point<std::chrono::system_clock> tTimestamp;
	typedef std::map<TimeStampFence,tTimestamp> tTimestamps;
	
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
};
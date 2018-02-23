#pragma once

// opencv
#include "opencv2/opencv.hpp"

// std
#include <map>
#include <chrono>

struct AreaOfInterest
{
	AreaOfInterest();
	
	bool operator<(const AreaOfInterest & pOther) const;
	
	cv::Point	mAABBMin;
	cv::Point	mAABBMax;
	int32_t		mPixelCount;
	cv::RotatedRect	mOBB;
	/** true if this area is partialy out of the frame
	*/
	bool		mOverlapBorder;
	/** true if this area is alone in it's vertical area,
	*	there is no other area sharing same x coordinates
	*/
	bool		mIsHorizontalySeparated;
};
typedef std::vector<AreaOfInterest> tAreas;


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
	
	tAreas & editAreas();

	void setTimestamp(TimeStampFence pFence);
	tTimestamp operator[](TimeStampFence pFence);
	
	static void ComputeLatencyAndFps(const tTimestamp & pPreviousTs, const tTimestamp & pNewTs, float & pLatency, float & pFps);

private:
	
	tLayers		mLayers;
	tTimestamps	mTimestamps;
	tAreas		mAreas;
};
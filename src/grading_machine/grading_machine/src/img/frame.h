#pragma once

// opencv
#include "opencv2/opencv.hpp"

// std
#include <map>
#include <chrono>

#include "frame_interface.h"

struct AreaOfInterest
{
	AreaOfInterest();
	
	bool operator<(const AreaOfInterest & pOther) const;
	
	cv::Point	mAABBMin;
	cv::Point	mAABBMax;
	int32_t		mPixelCount;
	cv::RotatedRect	mOBB;
	
	enum OverlapBorder
	{
		OB_None = 0x00,
		OB_XMin = 0x01,
		OB_XMax = 0x02,
		OB_YMin = 0x04,
		OB_YMax = 0x08
	};
	
	/** true if this area is partialy out of the frame
	*/
	int			mOverlapBorder;
	/** true if this area is alone in it's vertical area,
	*	there is no other area sharing same x coordinates
	*/
	bool		mIsAloneOnXAxis;
	
};
typedef std::vector<AreaOfInterest> tAreas;
typedef std::list<AreaOfInterest> tAreasList;


class Frame : public FrameInterface
{
public:
	
	Frame();
	virtual ~Frame();
	
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
	
	
	
	void swap(Frame & pOther);
	
	cv::Mat & operator[](Layer pLayer);
	const cv::Mat & operator[](Layer pLayer) const;
	
	tAreas & editAreas();
	const tAreas & getAreas() const;

	void setTimestamp(TimeStampFence pFence);
	tTimestamp operator[](TimeStampFence pFence);
	
	static void ComputeLatencyAndFps(const tTimestamp & pPreviousTs, const tTimestamp & pNewTs, float & pLatency, float & pFps);

	bool mExtractSuccessfull;
	
	virtual cv::Mat & editY();
	virtual cv::Mat & editU();
	virtual cv::Mat & editV();
	virtual void setGrabTimestamp();
private:
	
	tLayers		mLayers;
	tTimestamps	mTimestamps;
	tAreas		mAreas;
};
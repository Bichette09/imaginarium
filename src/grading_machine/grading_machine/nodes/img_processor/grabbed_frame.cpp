#include "grabbed_frame.h"

AreaOfInterest::AreaOfInterest()
	: mPixelCount(0)
	, mAABBMax(0,0)
	, mAABBMin(0,0)
	, mOverlapBorder(false)
	, mIsHorizontalySeparated(false)
{
}

bool AreaOfInterest::operator<(const AreaOfInterest & pOther) const
{
	return mAABBMax.y > pOther.mAABBMax.y;
}

GrabbedFrame::GrabbedFrame()
	: mExtractSuccessfull(false)
{
}

GrabbedFrame::~GrabbedFrame()
{
}

void GrabbedFrame::swap(GrabbedFrame & pOther)
{
	mLayers.swap(pOther.mLayers);
	mTimestamps.swap(pOther.mTimestamps);
	mAreas.swap(pOther.mAreas);
}

cv::Mat & GrabbedFrame::operator[](Layer pLayer)
{
	return mLayers[pLayer];
}

tAreas & GrabbedFrame::editAreas()
{
	return mAreas;
}

void GrabbedFrame::setTimestamp(TimeStampFence pFence)
{
	mTimestamps[pFence] = std::chrono::system_clock::now();
}

GrabbedFrame::tTimestamp GrabbedFrame::operator[](TimeStampFence pFence)
{
	return mTimestamps[pFence];
}

void GrabbedFrame::ComputeLatencyAndFps(const tTimestamp & pPreviousTs, const tTimestamp & pNewTs, float & pLatency, float & pFps)
{
	pLatency = std::chrono::duration<float>(std::chrono::system_clock::now() - pNewTs).count();
	float lElapsedTime = std::chrono::duration<float>(pNewTs-pPreviousTs).count();
	pFps = 0.;
	if(lElapsedTime > 0.)
		pFps = 1./ lElapsedTime;
}

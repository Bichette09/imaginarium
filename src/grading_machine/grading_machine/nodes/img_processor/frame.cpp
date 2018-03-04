#include "frame.h"

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

Frame::Frame()
	: mExtractSuccessfull(false)
{
}

Frame::~Frame()
{
}

void Frame::swap(Frame & pOther)
{
	mLayers.swap(pOther.mLayers);
	mTimestamps.swap(pOther.mTimestamps);
	mAreas.swap(pOther.mAreas);
}

cv::Mat & Frame::operator[](Layer pLayer)
{
	return mLayers[pLayer];
}

tAreas & Frame::editAreas()
{
	return mAreas;
}

void Frame::setTimestamp(TimeStampFence pFence)
{
	mTimestamps[pFence] = std::chrono::system_clock::now();
}

Frame::tTimestamp Frame::operator[](TimeStampFence pFence)
{
	return mTimestamps[pFence];
}

void Frame::ComputeLatencyAndFps(const tTimestamp & pPreviousTs, const tTimestamp & pNewTs, float & pLatency, float & pFps)
{
	pLatency = std::chrono::duration<float>(std::chrono::system_clock::now() - pNewTs).count();
	float lElapsedTime = std::chrono::duration<float>(pNewTs-pPreviousTs).count();
	pFps = 0.;
	if(lElapsedTime > 0.)
		pFps = 1./ lElapsedTime;
}

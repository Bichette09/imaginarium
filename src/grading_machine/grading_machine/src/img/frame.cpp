#include "frame.h"

AreaOfInterest::AreaOfInterest()
	: mPixelCount(0)
	, mAABBMax(0,0)
	, mAABBMin(0,0)
	, mOverlapBorder(OB_None)
	, mIsAloneOnXAxis(false)
{
}

bool AreaOfInterest::operator<(const AreaOfInterest & pOther) const
{
	if(mAABBMin.x == pOther.mAABBMin.x)
		return mAABBMin.y < pOther.mAABBMin.y;
	return mAABBMin.x < pOther.mAABBMin.x;
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

static cv::Mat sNullMat;

const cv::Mat & Frame::operator[](Layer pLayer) const
{
	tLayers::const_iterator lFindIt = mLayers.find(pLayer);
	if(lFindIt == mLayers.end())
	{
		return sNullMat;
	}
	return lFindIt->second;
}


tAreas & Frame::editAreas()
{
	return mAreas;
}

const tAreas & Frame::getAreas() const
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

cv::Mat & Frame::editY()
{
	return mLayers[Frame::Y];
}

cv::Mat & Frame::editU()
{
	return mLayers[Frame::U];
}

cv::Mat & Frame::editV()
{
	return mLayers[Frame::V];
}

void Frame::setGrabTimestamp()
{
	setTimestamp(Frame::F_GrabDone);
}
#include "light_and_line_frame.h"




LightAndLineFrame::LightAndLineFrame()
{
	for(int i = 0 ; i < LayerCount ; ++i)
	{
		mLayers[(Layer)i] = cv::Mat();
	}
}

LightAndLineFrame::~LightAndLineFrame()
{
}

void LightAndLineFrame::swap(LightAndLineFrame & pOther)
{
	mLayers.swap(pOther.mLayers);
	mTimestamps.swap(pOther.mTimestamps);
}

cv::Mat & LightAndLineFrame::operator[](Layer pLayer)
{
	return mLayers[pLayer];
}

static cv::Mat sNullMat;

const cv::Mat & LightAndLineFrame::operator[](Layer pLayer) const
{
	tLayers::const_iterator lFindIt = mLayers.find(pLayer);
	if(lFindIt == mLayers.end())
	{
		return sNullMat;
	}
	return lFindIt->second;
}

void LightAndLineFrame::setTimestamp(TimeStampFence pFence)
{
	mTimestamps[pFence] = std::chrono::system_clock::now();
}

LightAndLineFrame::tTimestamp LightAndLineFrame::operator[](TimeStampFence pFence)
{
	return mTimestamps[pFence];
}

cv::Mat & LightAndLineFrame::editY()
{
	return mLayers[LightAndLineFrame::Y];
}

cv::Mat & LightAndLineFrame::editU()
{
	return mLayers[LightAndLineFrame::U];
}

cv::Mat & LightAndLineFrame::editV()
{
	return mLayers[LightAndLineFrame::V];
}

void LightAndLineFrame::setGrabTimestamp()
{
	setTimestamp(LightAndLineFrame::F_GrabDone);
}
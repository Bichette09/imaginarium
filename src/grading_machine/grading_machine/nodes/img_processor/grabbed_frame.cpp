#include "grabbed_frame.h"

GrabbedFrame::GrabbedFrame()
{
}

GrabbedFrame::~GrabbedFrame()
{
}

void GrabbedFrame::swap(GrabbedFrame & pOther)
{
	mLayers.swap(pOther.mLayers);
	mTimestamps.swap(pOther.mTimestamps);
}

cv::Mat & GrabbedFrame::operator[](Layer pLayer)
{
	return mLayers[pLayer];
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

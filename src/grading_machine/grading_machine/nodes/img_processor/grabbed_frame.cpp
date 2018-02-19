#include "grabbed_frame.h"

GrabbedFrame::GrabbedFrame()
{
}

GrabbedFrame::~GrabbedFrame()
{
}

void GrabbedFrame::swap(GrabbedFrame & pOther)
{
	tTimestamp lTs = mGrabTs;
	mGrabTs = pOther.mGrabTs;
	pOther.mGrabTs = lTs;
	mLayers.swap(pOther.mLayers);
	
}

cv::Mat & GrabbedFrame::operator[](Layer pLayer)
{
	return mLayers[pLayer];
}

void GrabbedFrame::setTimestamp()
{
	mGrabTs = std::chrono::system_clock::now();
}

GrabbedFrame::tTimestamp GrabbedFrame::getTimestamp() const
{
	return mGrabTs;
}

void GrabbedFrame::ComputeLatencyAndFps(const tTimestamp & pPreviousTs, const tTimestamp & pNewTs, float & pLatency, float & pFps)
{
	pLatency = std::chrono::duration<float>(std::chrono::system_clock::now() - pNewTs).count();
	float lElapsedTime = std::chrono::duration<float>(pNewTs-pPreviousTs).count();
	pFps = 0.;
	if(lElapsedTime > 0.)
		pFps = 1./ lElapsedTime;
}

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
	
	for(int i = 0 ; i < LC_Count ; ++i)
	{
		mColorAreas[i].swap(pOther.mColorAreas[i]);
	}
	std::swap(mLightSearchArea, pOther.mLightSearchArea);
	std::swap(mLineSearchArea, pOther.mLineSearchArea);
	for(int i = 0 ; i < LT_Count ; ++i)
	{
		mLines[i].swap(pOther.mLines[i]);
	}
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

LightAndLineFrame::tRects & LightAndLineFrame::operator[](ColorAreas pType)
{
	return mColorAreas[pType];
}

const LightAndLineFrame::tRects & LightAndLineFrame::operator[](ColorAreas pType) const
{
	return mColorAreas[pType];
}

LightAndLineFrame::tLines & LightAndLineFrame::operator[](LinesType pType)
{
	return mLines[pType];
}

const LightAndLineFrame::tLines & LightAndLineFrame::operator[](LinesType pType) const
{
	return mLines[pType];
}

cv::Rect & LightAndLineFrame::editLightSearchArea()
{
	return mLightSearchArea;
}

const cv::Rect & LightAndLineFrame::getLightSearchArea() const
{
	return mLightSearchArea;
}


cv::Rect & LightAndLineFrame::editLineSearchArea()
{
	return mLineSearchArea;
}

const cv::Rect & LightAndLineFrame::getLineSearchArea() const
{
	return mLineSearchArea;
}
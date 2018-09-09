#include "light_detector.h"

// std
#include <algorithm>

// ros
#include "ros/ros.h"

LightDetector::LightDetector(int pWidthResolution, int pHeightResolution)
	: mHeight(pHeightResolution)
	, mWidth(pWidthResolution)
	, mCreationTime(std::chrono::system_clock::now())
{
	clearDetector();
}

LightDetector::~LightDetector()
{
}

void LightDetector::addNewFrame(const LightAndLineFrame & pFrame)
{
	if(mLightSearchRoi != pFrame.getLightSearchArea())
	{
		clearDetector();
	}
	mLightSearchRoi = pFrame.getLightSearchArea();
	const std::chrono::time_point<std::chrono::system_clock> lNow(std::chrono::system_clock::now());
	mCurrentTs = std::chrono::duration_cast<std::chrono::milliseconds>(lNow-mCreationTime).count();
	
	addDetectedAreas(pFrame[LightAndLineFrame::LC_Red],LightAndLineFrame::LC_Red);
	addDetectedAreas(pFrame[LightAndLineFrame::LC_Yellow],LightAndLineFrame::LC_Yellow);
	addDetectedAreas(pFrame[LightAndLineFrame::LC_Blue],LightAndLineFrame::LC_Blue);
}

void LightDetector::addDetectedAreas(const tRects & pAreas, LightAndLineFrame::LightColor pColor)
{
	tRects::const_iterator lIt = pAreas.begin();
	const tRects::const_iterator lItEnd = pAreas.end();
	for( ; lIt != lItEnd ; ++lIt)
	{
		int lXMin = lIt->x - mLightSearchRoi.x;
		int lXMax = lXMin + lIt->width;
		int lYMin = lIt->y - mLightSearchRoi.y;
		int lYMax = lYMin + lIt->height;
		
		lXMin = std::max(0,(lXMin * mWidth) / mLightSearchRoi.width);
		lXMax = std::min(mWidth-1,(lXMax * mWidth) / mLightSearchRoi.width);
		lYMin = std::max(0,(lYMin * mHeight) / mLightSearchRoi.height);
		lYMax = std::min(mHeight-1,(lYMax * mHeight) / mLightSearchRoi.height);
		
		for(int x = lXMin ; x <= lXMax ; ++x)
		{
			tCellVector & lColumn = mCellMatrix[x];
			for(int y = lYMin; y <= lYMax ; ++y)
			{
				lColumn[y].mTs[pColor] = mCurrentTs;
			}
		}
	}
}

void LightDetector::clearDetector()
{
	mCellMatrix.clear();
	mCellMatrix.resize(mWidth);
	tCellMatrix::iterator lIt = mCellMatrix.begin();
	const tCellMatrix::iterator lItEnd = mCellMatrix.end();
	for( ; lIt != lItEnd ; ++lIt)
	{
		lIt->resize(mHeight);
	}
	mCurrentTs = 0;
	mLightSearchRoi = cv::Rect();
}

bool LightDetector::detectLightSequence(tTs pTimeWindowSec)
{
	const tTs lMinTsValue = mCurrentTs - pTimeWindowSec*1000;
	int lTs[LightAndLineFrame::LC_Count];
	for(int x = 0 ; x < mWidth ; ++x)
	{
		tCellVector & lColumn = mCellMatrix[x];
		memset(lTs,0,sizeof(lTs));
		
		// iterate from top to bottom and look for light sequence
		for(int y = 0; y < mHeight ; ++y)
		{
			const DetectionCell & lCell = lColumn[y];
			
			// check for red on this cell
			if(lCell.mTs[LightAndLineFrame::LC_Red] > lMinTsValue)
			{
				// got red ! update red ts and clear yellow and blue
				lTs[LightAndLineFrame::LC_Red] = std::max(lTs[LightAndLineFrame::LC_Red],lCell.mTs[LightAndLineFrame::LC_Red]);
				lTs[LightAndLineFrame::LC_Blue] = 0;
				lTs[LightAndLineFrame::LC_Yellow] = 0;
			}
			
			// check for yellow, it should be newer than red
			if( (lCell.mTs[LightAndLineFrame::LC_Yellow] > lMinTsValue) && (lCell.mTs[LightAndLineFrame::LC_Yellow] > lCell.mTs[LightAndLineFrame::LC_Red]) && (lTs[LightAndLineFrame::LC_Red] > lMinTsValue))
			{
				lTs[LightAndLineFrame::LC_Yellow] = std::max(lTs[LightAndLineFrame::LC_Yellow],lCell.mTs[LightAndLineFrame::LC_Yellow]);
				lTs[LightAndLineFrame::LC_Blue] = 0;
			}
			
			// check for blue, it should be newer than yellow
			if((lCell.mTs[LightAndLineFrame::LC_Blue] > lMinTsValue) && (lCell.mTs[LightAndLineFrame::LC_Blue] > lCell.mTs[LightAndLineFrame::LC_Yellow]) && (lTs[LightAndLineFrame::LC_Yellow] > lMinTsValue))
			{
				// for blue we store 
				lTs[LightAndLineFrame::LC_Blue] = std::max(lTs[LightAndLineFrame::LC_Blue],lCell.mTs[LightAndLineFrame::LC_Blue]);
				
				// ensure that blue is litup since enough time
				if( (mCurrentTs - lTs[LightAndLineFrame::LC_Yellow]) > 1000)
				{
					// GOGOGOGO
					return true;
				}
			}
		}
	}
	return false;
}

void LightDetector::createDebugImg(cv::Mat & pTarget,tTs pTimeWindowSec)
{
	const tTs lMinTsValue = mCurrentTs - pTimeWindowSec*1000;
	
	pTarget = cv::Mat(mHeight,mWidth,CV_8UC3,cv::Scalar(0,0,0));
	for(int x = 0 ; x < mWidth ; ++x)
	{
		tCellVector & lColumn = mCellMatrix[x];
		for(int y = 0; y < mHeight ; ++y)
		{
			cv::Scalar lColor(0,0,0);
			for(int c = 0 ; c < 3 ; ++c)
			{
				if(lColumn[y].mTs[c] == 0)
					continue;
				if(lColumn[y].mTs[c] < lMinTsValue)
				{
					lColor[c] = 32;
				}
				else
				{
					lColor[c] = 128 + ((lColumn[y].mTs[c] - lMinTsValue) * 128) / (pTimeWindowSec*1000); 
				}
			}
			
			if(lColor != cv::Scalar(0,0,0))
			{
				cv::rectangle(pTarget,cv::Rect(x,y,1,1),lColor,1);
			}
		}
	}
}

LightDetector::DetectionCell::DetectionCell()
{
	memset(&mTs,0,sizeof(mTs));
}


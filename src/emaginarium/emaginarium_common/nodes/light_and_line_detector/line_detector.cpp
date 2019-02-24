#include "line_detector.h"

// std
#include <algorithm>

// ros
#include "ros/ros.h"

LineDetector::DetectionCell::DetectionCell()
	: mNoDataInLastFrame(true)
{
}

void LineDetector::DetectionCell::clearDetectionCell()
{
	mTsQueue.clear();
	mNoDataInLastFrame = true;
}

LineDetector::LineDetector(int pWidthResolution)
	: mWidth(pWidthResolution)
	, mHeight(1)
	, mCreationTime(std::chrono::system_clock::now())
{
	clearDetector();
}

LineDetector::~LineDetector()
{
}

void LineDetector::addNewFrame(const LightAndLineFrame & pFrame,tTs pTimeWindowMSec)
{
	if(mLineSearchRoi != pFrame.getLineSearchArea())
	{
		clearDetector();
	}
	mLineSearchRoi = pFrame.getLineSearchArea();
	
	
	const std::chrono::time_point<std::chrono::system_clock> lNow(std::chrono::system_clock::now());
	mCurrentTs = std::chrono::duration_cast<std::chrono::milliseconds>(lNow-mCreationTime).count();
	
	addLineColorAreas(pFrame,LightAndLineFrame::LC_LineGreen,LightAndLineFrame::LT_Green,pTimeWindowMSec);
	addLineColorAreas(pFrame,LightAndLineFrame::LC_LineRed,LightAndLineFrame::LT_Red,pTimeWindowMSec);
}


void LineDetector::clearDetector()
{
	for(int i = 0 ; i < LightAndLineFrame::LT_Count ; ++i)
	{
		mDetectionCells[i].clear();
		mDetectionCells[i].resize(mWidth);
	}
	mCurrentTs = 0;
	mLineSearchRoi = cv::Rect();
}

void LineDetector::addLineColorAreas(const LightAndLineFrame & pFrame, LightAndLineFrame::ColorAreas pColorArea, LightAndLineFrame::LinesType pLineType,tTs pTimeWindowMSec)
{
	tDetectionCells & lCells = mDetectionCells[pLineType];
	{
		tTs lLimitTs = mCurrentTs - pTimeWindowMSec;
		size_t lMaxEntriesInList = 250;
		tDetectionCells::iterator lIt = lCells.begin();
		tDetectionCells::iterator lItEnd = lCells.end();
		for( ; lIt != lItEnd ; ++lIt)
		{
			lIt->mNoDataInLastFrame = true;
			while( (lIt->mTsQueue.size() > lMaxEntriesInList) && (!lIt->mTsQueue.empty()) && (lIt->mTsQueue.front() < lLimitTs))
			{
				lIt->mTsQueue.pop_front();
			}
		}
	}
	
	{
		const tRects & lAreas = pFrame[pColorArea];
		tRects::const_iterator lIt = lAreas.begin();
		const tRects::const_iterator lItEnd = lAreas.end();
		for( ; lIt != lItEnd ; ++lIt)
		{
			int lXMin = lIt->x - mLineSearchRoi.x;
			int lXMax = lXMin + lIt->width;
			int lYMin = lIt->y - mLineSearchRoi.y;
			int lYMax = lYMin + lIt->height;
			
			lXMin = std::max(0,(lXMin * mWidth) / mLineSearchRoi.width);
			lXMax = std::min(mWidth-1,(lXMax * mWidth) / mLineSearchRoi.width);
			lYMin = std::max(0,(lYMin * mHeight) / mLineSearchRoi.height);
			lYMax = std::min(mHeight-1,(lYMax * mHeight) / mLineSearchRoi.height);
			
			for(int x = lXMin ; x <= lXMax ; ++x)
			{
				DetectionCell & lCell = lCells[x];
				lCell.mTsQueue.push_back(mCurrentTs);
				lCell.mNoDataInLastFrame = false;
			}
		}
	}
}

bool LineDetector::detectLine(LightAndLineFrame::LinesType pLineType)
{
	if(mCurrentTs <= 0)
		return false;
		
	tDetectionCells & lCells = mDetectionCells[pLineType];
	
	int lCellOk = 0;
	int lCellWithoutData = 0;
	int lCellNotEmpty = 0;
	tDetectionCells::const_iterator lIt = lCells.begin();
	tDetectionCells::const_iterator lItEnd = lCells.end();
	for( ; lIt != lItEnd ; ++lIt)
	{
		if(lIt->mNoDataInLastFrame)
		{
			++lCellWithoutData;
		}
		if(!lIt->mTsQueue.empty())
		{
			++lCellNotEmpty;
		}
		
		if(lIt->mNoDataInLastFrame && !lIt->mTsQueue.empty())
		{
			++lCellOk;
		}
	}
	//ROS_WARN_STREAM(lCellOk<<" " <<lCellWithoutData<<" "<<lCellNotEmpty);
	return lCellOk > ((lCells.size() * 2 / 3));
}

void LineDetector::createDebugImg(cv::Mat & pTarget,tTs pTimeWindowMSec)
{
	const tTs lMinTsValue = mCurrentTs - pTimeWindowMSec;
	
	
	pTarget = cv::Mat(10,mWidth,CV_8UC3,cv::Scalar(0,0,0));
	for(int i = 0 ; i < mWidth ; ++i)
	{
		cv::Scalar lColor(0,0,0);
		
		if(!mDetectionCells[LightAndLineFrame::LT_Red][i].mTsQueue.empty())
		{
			if(mDetectionCells[LightAndLineFrame::LT_Red][i].mNoDataInLastFrame)
			{
				lColor[2] = 64;
			}
			else
			{
				lColor[2] = 255;
			}
		}
		
		
		if(!mDetectionCells[LightAndLineFrame::LT_Green][i].mTsQueue.empty())
		{
			if(mDetectionCells[LightAndLineFrame::LT_Green][i].mNoDataInLastFrame)
			{
				lColor[1] = 64;
			}
			else
			{
				lColor[1] = 255;
			}
		}
		
		if(lColor != cv::Scalar(0,0,0))
		{
			cv::rectangle(pTarget,cv::Rect(i,0,1,10),lColor,1);
		}
	}
	
}



#include "extract_thread.h"

#include <unistd.h>

ExtractThread::Parameters::Parameters()
	: mConnectivityFullWay(false)
	, mMinimumPixelsPerGroup(128)
{
}

ExtractThread::ExtractThread(FilterThread & pFilterThread, const Parameters & pParameters)
	: mFilterThread(pFilterThread)
	, mQuit(false)
	, mGotNewFrame(false)
	, mParameters(pParameters)
	
{
	startThread();
}

ExtractThread::~ExtractThread()
{
	{
		std::unique_lock<std::mutex> lLock(mMutex);
		mQuit = true;
		mWaitCondition.notify_all();
	}
	waitThread();
}

bool ExtractThread::getNextFrame(GrabbedFrame & pFrame,int pTimeoutMs)
{
	std::unique_lock<std::mutex> lLock(mMutex);
	if(!mGotNewFrame)
	{
		mWaitCondition.wait_until(lLock,std::chrono::system_clock::now() + std::chrono::milliseconds(pTimeoutMs));
	}
	if(!mGotNewFrame)
		return false;
	mFrame.swap(pFrame);
	mGotNewFrame = false;
	mWaitCondition.notify_all();
	return true;
}

void ExtractThread::run()
{
	GrabbedFrame lFrame;

	cv::Mat lLabels, lStats, lCentroids;
	cv::Mat lNonZeroCoordinates;
	
	while(!mQuit)
	{
		if(!mFilterThread.getNextFrame(lFrame))
			continue;
		lFrame.setTimestamp(GrabbedFrame::F_AreaExtractionStart);
		tAreas & lAreas = lFrame.editAreas();
		lAreas.clear();

		int lGroupCount = cv::connectedComponentsWithStats (
			lFrame[GrabbedFrame::BackgroundMask],
			lLabels,
			lStats,
			lCentroids,
			mParameters.mConnectivityFullWay ? 8 : 4,
			CV_16U,
			cv::CCL_DEFAULT);
		
		if(1 < lGroupCount && lGroupCount <= 32)
		{
			lAreas.reserve(lGroupCount - 1);
			for(int i = 1 ; i < lGroupCount ; ++i)
			{
				AreaOfInterest lArea;
				lArea.mAABBMin.x = lStats.at<int32_t>(i,cv::CC_STAT_LEFT);
				lArea.mAABBMin.y = lStats.at<int32_t>(i,cv::CC_STAT_TOP);
				lArea.mAABBMax.x = lArea.mAABBMin.x + lStats.at<int32_t>(i,cv::CC_STAT_WIDTH);
				lArea.mAABBMax.y = lArea.mAABBMin.y + lStats.at<int32_t>(i,cv::CC_STAT_HEIGHT);
				lArea.mPixelCount = lStats.at<int32_t>(i, cv::CC_STAT_AREA);
				
				if(lArea.mPixelCount < mParameters.mMinimumPixelsPerGroup)
					continue;
				
				cv::Mat lSubMat(lFrame[GrabbedFrame::BackgroundMask],cv::Range(lArea.mAABBMin.y,lArea.mAABBMax.y),cv::Range(lArea.mAABBMin.x,lArea.mAABBMax.x));
				
				cv::findNonZero(lSubMat, lNonZeroCoordinates);
				lArea.mOBB = cv::minAreaRect(lNonZeroCoordinates);
				lArea.mOBB.center.x += lArea.mAABBMin.x;
				lArea.mOBB.center.y += lArea.mAABBMin.y;
				lAreas.push_back(lArea);
			}
			std::sort(lAreas.begin(),lAreas.end());
			
			//TODO check border and horizontal overlap
			
		}

		
		lFrame.setTimestamp(GrabbedFrame::F_AreaExtractionDone);
		
		{
			std::unique_lock<std::mutex> lLock(mMutex);
			if(mQuit)
				continue;
			mFrame.swap(lFrame);
			mGotNewFrame = true;
			mWaitCondition.notify_all();
			// and wait until frame is consumed
			mWaitCondition.wait(lLock);
		}
		
	}
}
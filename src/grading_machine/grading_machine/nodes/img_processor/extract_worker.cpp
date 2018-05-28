#include "extract_worker.h"

#include "filter_worker.h"

#include <unistd.h>

ExtractWorker::Parameters::Parameters()
	: mConnectivityFullWay(false)
	, mMinimumPixelsPerGroup(128)
	, mMinimumBackgroundPercent(0.5)
	, mMinimumSpaceBetweenAreaPercent(0.05)
	, mMaxAreaToExtract(10)
{
}

ExtractWorker::ExtractWorker(FilterWorker & pFilterWorker, const Parameters & pParameters)
	: mFilterThread(NULL)
	, mFilterWorker(pFilterWorker)
	, mParameters(pParameters)
	
{
	mFilterThread = new FrameProcessor(mFilterWorker);
}

ExtractWorker::~ExtractWorker()
{
	delete mFilterThread; mFilterThread = NULL;
}


bool ExtractWorker::computeNextResult(Frame & pRes)
{
	if(!mFilterThread->getNextFrame(pRes))
		return false;
	
	const int lWidth = pRes[Frame::BackgroundMask].cols;
	const int lHeight = pRes[Frame::BackgroundMask].rows;

	pRes.setTimestamp(Frame::F_AreaExtractionStart);
	tAreas & lAreas = pRes.editAreas();
	lAreas.clear();
	const int lNonZeroPx = cv::countNonZero(pRes[Frame::BackgroundMask]);
	const int lTotalPxCount = lWidth * lHeight;
	pRes.mExtractSuccessfull = false;
	if( (lTotalPxCount - lNonZeroPx) >= (mParameters.mMinimumBackgroundPercent*lTotalPxCount))
	{
		int lGroupCount = cv::connectedComponentsWithStats (
			pRes[Frame::BackgroundMask],
			mLabels,
			mStats,
			mCentroids,
			mParameters.mConnectivityFullWay ? 8 : 4,
			CV_16U,
			cv::CCL_DEFAULT);
		if(1 < lGroupCount && lGroupCount <= mParameters.mMaxAreaToExtract)
		{
			lAreas.reserve(lGroupCount - 1);
			for(int i = 1 ; i < lGroupCount ; ++i)
			{
				AreaOfInterest lArea;
				lArea.mAABBMin.x = mStats.at<int32_t>(i,cv::CC_STAT_LEFT);
				lArea.mAABBMin.y = mStats.at<int32_t>(i,cv::CC_STAT_TOP);
				lArea.mAABBMax.x = lArea.mAABBMin.x + mStats.at<int32_t>(i,cv::CC_STAT_WIDTH);
				lArea.mAABBMax.y = lArea.mAABBMin.y + mStats.at<int32_t>(i,cv::CC_STAT_HEIGHT);
				lArea.mPixelCount = mStats.at<int32_t>(i, cv::CC_STAT_AREA);
				
				if(lArea.mPixelCount < mParameters.mMinimumPixelsPerGroup)
					continue;
				
				cv::Mat lSubMat(pRes[Frame::BackgroundMask],cv::Range(lArea.mAABBMin.y,lArea.mAABBMax.y),cv::Range(lArea.mAABBMin.x,lArea.mAABBMax.x));
				
				cv::findNonZero(lSubMat, mNonZeroCoordinates);
				lArea.mOBB = cv::minAreaRect(mNonZeroCoordinates);
				lArea.mOBB.center.x += lArea.mAABBMin.x;
				lArea.mOBB.center.y += lArea.mAABBMin.y;
				lAreas.push_back(lArea);
			}
			std::sort(lAreas.begin(),lAreas.end());
			
			tAreas::iterator lIt = lAreas.begin();
			const tAreas::iterator lItEnd = lAreas.end();
			for( ; lIt != lItEnd ; ++lIt)
			{
				AreaOfInterest & lArea = *lIt;
				lArea.mIsAloneOnXAxis = true;
				
				if(lIt != lAreas.begin())
				{
					AreaOfInterest & lPrevious = *(lIt - 1);
					int lDeltaX = lArea.mAABBMin.x - lPrevious.mAABBMax.x;
					if(lDeltaX < (lWidth * mParameters.mMinimumSpaceBetweenAreaPercent))
					{
						lArea.mIsAloneOnXAxis = false;
						lPrevious.mIsAloneOnXAxis = false;
					}
				}
				
				if(lArea.mAABBMin.x <= 1)
					lArea.mOverlapBorder |= AreaOfInterest::OB_XMin;
				if(lArea.mAABBMax.x >= (lWidth - 2))
					lArea.mOverlapBorder |= AreaOfInterest::OB_XMax;
				if(lArea.mAABBMin.y <= (int)((lHeight * mFilterWorker.mParameters.mExclusionZoneTopPercent) + 1))
					lArea.mOverlapBorder |= AreaOfInterest::OB_YMin;
				if(lArea.mAABBMax.y >= (int)(lHeight - 2 - (int)(lHeight * mFilterWorker.mParameters.mExclusionZoneBottomPercent)))
					lArea.mOverlapBorder |= AreaOfInterest::OB_YMax;
			}
			
			pRes.mExtractSuccessfull = true;
		}
	}

	pRes.setTimestamp(Frame::F_AreaExtractionDone);

	return true;
}
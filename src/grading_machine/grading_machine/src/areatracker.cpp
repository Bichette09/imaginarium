#include "areatracker.h"

// ros
#include <ros/ros.h>

#include "scalaraggregator.h"



AreaTracker::AreaTrackingFunctor::AreaTrackingFunctor()
{
}

AreaTracker::AreaTrackingFunctor::~AreaTrackingFunctor()
{
}

void AreaTracker::AreaTrackingFunctor::clearAreas()
{
}

AreaTracker::AreaTrackingFunctor::tAreaId AreaTracker::AreaTrackingFunctor::getNewAreaId()
{
	return 0;
}

void AreaTracker::AreaTrackingFunctor::updateArea(tAreaId pId, int pIndexInFrame, const Frame & pFrame)
{
}

void AreaTracker::AreaTrackingFunctor::releaseArea(tAreaId pId)
{
}

AreaTracker::AreaTracker()
	: mFunctor(NULL)
{
	// build offset vector
	// first try it : no area enter/exit
	mOffsets.push_back(0);
	
	for(int i = 1 ; i <= 5 ; ++i)
	{
		mOffsets.push_back(i);
		mOffsets.push_back(-i);
	}
	
	setTrackingFunctor(NULL);
}

AreaTracker::~AreaTracker()
{
	
}

void AreaTracker::clear()
{
	mPreviousAreas.clear();
	mIds.clear();
	if(mFunctor)
		mFunctor->clearAreas();
}

void AreaTracker::setTrackingFunctor(AreaTrackingFunctor * pFunctor)
{
	mFunctor = pFunctor;
	clear();
}




void AreaTracker::addNewFrame(const Frame & pFrame)
{
	if(!mFunctor)
		return;
	
	const tAreas & lNewAreas = pFrame.getAreas();
	if(mPreviousAreas.empty() || lNewAreas.empty())
	{
		// quick exit, there is nothing to track
		{
			tIntVector::iterator lIdIt = mIds.begin();
			tAreas::const_iterator lAreaIt = mPreviousAreas.begin();
			const tAreas::const_iterator lAreaItEnd = mPreviousAreas.end();
			for( ; lAreaIt != lAreaItEnd ; ++lAreaIt, ++lIdIt)
			{
				mFunctor->releaseArea(*lIdIt);
			}
			mIds.clear();
			mPreviousAreas.clear();
		}

		{
			mPreviousAreas = lNewAreas;
			tAreas::const_iterator lAreaIt = mPreviousAreas.begin();
			const tAreas::const_iterator lAreaItEnd = mPreviousAreas.end();
			for( ; lAreaIt != lAreaItEnd ; ++lAreaIt)
			{
				mIds.push_back(mFunctor->getNewAreaId());
				mFunctor->updateArea(mIds.back(),lAreaIt - mPreviousAreas.begin(),pFrame);
			}
		}
		
		return;
	}
	
	
	// input is ordered along x axis
	// we assume that areas will all move at the same speed
	// we will try different offset to simulate area enter/exit
	const int lWidth = pFrame[Frame::BackgroundMask].cols;
	const int lHeight = pFrame[Frame::BackgroundMask].rows;
	const float lMaxDistributionXSize = lWidth * 0.15;
	const float lMaxDistributionYSize = lHeight * 0.05;
	const float lMaxMovement = lWidth * 0.33;
	int 	lCurrentOffset = 0;
	float 	lCurrentDist = std::numeric_limits<float>::infinity();
	
#define DEBUG_TRACKING
#ifdef DEBUG_TRACKING
	std::stringstream lDebug;
#endif

	tIntVector::iterator lOffsetIt = mOffsets.begin();
	const tIntVector::iterator lOffsetItEnd = mOffsets.end();
	for( ; lOffsetIt != lOffsetItEnd ; ++lOffsetIt)
	{
		ScalarAggregator<float> lXMetric;
		ScalarAggregator<float> lYMetric;
		
#ifdef DEBUG_TRACKING
		lDebug<<"\nOffset "<<*lOffsetIt;
#endif
		
		// we will check new[n] with previous[n+offset]
		tAreas::const_iterator lAreaIt = lNewAreas.begin();
		const tAreas::const_iterator lAreaItEnd = lNewAreas.end();
		int lIndexPrevious = *lOffsetIt;
		for(; lAreaIt != lAreaItEnd && lIndexPrevious < mPreviousAreas.size(); ++lAreaIt, ++lIndexPrevious)
		{
			if(lIndexPrevious < 0)
				continue;
			
			
			const AreaOfInterest & lNewArea = *lAreaIt;
			const AreaOfInterest & lPreviousArea = mPreviousAreas[lIndexPrevious];
			
			
			// we compute delta on x axis
			float lXDelta = 0.;
			if( (lNewArea.mOverlapBorder & AreaOfInterest::OB_XMin) || (lPreviousArea.mOverlapBorder & AreaOfInterest::OB_XMin))
			{
				// use aabb max
				lXDelta = lNewArea.mAABBMax.x - lPreviousArea.mAABBMax.x;
			}
			else
			{
				// use aabb min
				lXDelta = lNewArea.mAABBMin.x - lPreviousArea.mAABBMin.x;
			}
			// y metric should remain close to zero so we could use min or max
			float lYDelta = lNewArea.mAABBMax.y - lPreviousArea.mAABBMax.y;
			lXMetric.addValue(lXDelta);
			lYMetric.addValue(lYDelta);
#ifdef DEBUG_TRACKING
			lDebug<<"\nNew["<<(lAreaIt - lNewAreas.begin())<<"] "<<lNewArea.mOBB.center.x<<" Prev["<<lIndexPrevious<<"] "<<lPreviousArea.mOBB.center.x<<" => "<<lXDelta<<" "<<lYDelta;
#endif
		}
		
		if(lXMetric.mAggCount == 0)
		{
			continue;
		}
		
		// if min/max delta is too big it might denote that this is not a valid combination
		if( (lXMetric.mMax - lXMetric.mMin) > lMaxDistributionXSize)
		{
			continue;
		}
		
		// if y metric is too big ignore this
		if( (lYMetric.mMax - lYMetric.mMin) > lMaxDistributionYSize)
		{
			continue;
		}
		
		if( std::abs(lXMetric.mAvg) > std::abs(lCurrentDist) || std::abs(lXMetric.mAvg) > lMaxMovement)
		{
			continue;
		}
		lCurrentDist = lXMetric.mAvg;
		lCurrentOffset = *lOffsetIt;
	} // for each offset

	if(lCurrentDist == std::numeric_limits<float>::infinity())
	{
		// fail to track
		ROS_ERROR_STREAM("Tracking error !! ");
#ifdef DEBUG_TRACKING
		ROS_ERROR_STREAM(lDebug.str());
#endif
		clear();
		return;
	}
	
	
	for(int i = 0 ; i < mPreviousAreas.size() ; ++i)
	{
		int lIndexInNew = i - lCurrentOffset;
		if(lIndexInNew < 0 || lIndexInNew >= lNewAreas.size())
		{
			mFunctor->releaseArea(mIds[i]);
		}
	}
	
	tIntVector lNewIds;
	lNewIds.reserve(lNewAreas.size());
	for(int i = 0 ; i < lNewAreas.size() ; ++i)
	{
		int lIndexInPrevious = i + lCurrentOffset;
		if(lIndexInPrevious < 0 || lIndexInPrevious >= mPreviousAreas.size())
		{
			// new area
			lNewIds.push_back(mFunctor->getNewAreaId());
		}
		else
		{
			lNewIds.push_back(mIds[lIndexInPrevious]);
		}
		mFunctor->updateArea(lNewIds.back(),i,pFrame);
	}
	
	//std::cout<<lCurrentDist<< " "<<lCurrentOffset<<std::endl;
	mIds.swap(lNewIds);
	mPreviousAreas = lNewAreas;
	
	
	
	
}

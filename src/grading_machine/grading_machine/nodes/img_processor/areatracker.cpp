#include "areatracker.h"

#include "scalaraggregator.h"

AreaTracker::AreaTrackingFunctor::AreaTrackingFunctor()
{
}

AreaTracker::AreaTrackingFunctor::~AreaTrackingFunctor()
{
}

void AreaTracker::AreaTrackingFunctor::onAreaEnter(const Frame & pFrame, const AreaOfInterest & pArea)
{
}

void AreaTracker::AreaTrackingFunctor::onAreaMove(const Frame & pFrame, const AreaOfInterest & pArea)
{
}

void AreaTracker::AreaTrackingFunctor::onAreaExit(const AreaOfInterest & pPreviousArea)
{
}

AreaTracker::AreaTrackingFunctor * AreaTracker::AreaTrackingFunctor::copy() const
{
	return new AreaTrackingFunctor();
}

AreaTracker::AreaTracker()
{
	// build offset vector
	// first try it : no area enter/exit
	mOffsets.push_back(0);
	
	// for(int i = 1 ; i <= 5 ; ++i)
	// {
		// mOffsets.push_back(i);
		// mOffsets.push_back(-i);
	// }
	
	setTrackingFunctor(NULL);
}

AreaTracker::~AreaTracker()
{
	
}

void AreaTracker::clear()
{
	mPreviousAreas.clear();
	mFunctors.clear();
}

void AreaTracker::setTrackingFunctor(std::unique_ptr<AreaTrackingFunctor> pFunctor)
{
	mFunctorTemplate.swap(pFunctor);
	if(!mFunctorTemplate)
	{
		mFunctorTemplate.reset(new AreaTrackingFunctor());
	}
}




void AreaTracker::addNewFrame(const Frame & pFrame)
{
	const tAreas & lNewAreas = pFrame.getAreas();
	if(mPreviousAreas.empty() || lNewAreas.empty())
	{
		// quick exit, there is nothing to track
		{
			tFunctors::iterator lFunctorIt = mFunctors.begin();
			tAreas::const_iterator lAreaIt = mPreviousAreas.begin();
			const tAreas::const_iterator lAreaItEnd = mPreviousAreas.end();
			for( ; lAreaIt != lAreaItEnd ; ++lAreaIt, ++lFunctorIt)
			{
				(*lFunctorIt)->onAreaExit(*lAreaIt);
			}
			mFunctors.clear();
			mPreviousAreas.clear();
		}

		{
			mPreviousAreas = lNewAreas;
			tAreas::const_iterator lAreaIt = mPreviousAreas.begin();
			const tAreas::const_iterator lAreaItEnd = mPreviousAreas.end();
			for( ; lAreaIt != lAreaItEnd ; ++lAreaIt)
			{
				tFunctors::value_type lFunctor(mFunctorTemplate->copy());
				lFunctor->onAreaEnter(pFrame,*lAreaIt);
				mFunctors.push_back(lFunctor);
			}
		}
		
		return;
	}
	
	{
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
				lDebug<<"\nNew["<<(lAreaIt - lNewAreas.begin())<<"] "<<lNewArea.mOBB.center.x<<" Prev["<<lIndexPrevious<<"] "<<lPreviousArea.mOBB.center.x<<" => "<<lDelta<<" "<<lYDelta;
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
		std::cout<<lCurrentDist<< " "<<lCurrentOffset<<std::endl;
		mPreviousAreas = lNewAreas;
	}
	
	
	
}

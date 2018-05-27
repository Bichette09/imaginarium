#include "areatracker.h"


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
	
	// input is ordered along x axis
	// we assume that areas will all move at the same speed
	// we will try different offset to simulate area enter/exit
	
	int lOffset = 0;
	
	// calculer delta moyen (deplacement) et min/max => taux de variabilité
	
	
	
	
	
}

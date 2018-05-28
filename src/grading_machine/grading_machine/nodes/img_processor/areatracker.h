#pragma once

// std
#include <map>
#include <chrono>
#include <memory>

// ros
#include "ros/ros.h"

#include "frame.h"

/** this class will compute area movement between two frames to allow tracking area and ensure
*	that they always match the same model
*/
class AreaTracker
{
public:
	AreaTracker();
	~AreaTracker();
	
	class AreaTrackingFunctor
	{
	public:
		AreaTrackingFunctor();
		virtual ~AreaTrackingFunctor();
		
		/** this method is called when tracker detect 
		*/
		virtual void onAreaEnter(const Frame & pFrame, const AreaOfInterest & pArea);
		/** this method is called when area has moved
		*/
		virtual void onAreaMove(const Frame & pFrame, const AreaOfInterest & pArea);
		/** this method is called when area exit
		*/
		virtual void onAreaExit(const AreaOfInterest & pPreviousArea);
		
		virtual AreaTrackingFunctor * copy() const;
	};
	
	
	
	void clear();
	void setTrackingFunctor(std::unique_ptr<AreaTrackingFunctor> pFunctor);
	void addNewFrame(const Frame & pFrame);
	
	
private:
	typedef std::vector<std::shared_ptr<AreaTrackingFunctor> > tFunctors;
	typedef std::vector<int> tIntVector;
	tIntVector 	mOffsets;

	std::unique_ptr<AreaTrackingFunctor> mFunctorTemplate;
	tFunctors	mFunctors;
	tAreas		mPreviousAreas;
};

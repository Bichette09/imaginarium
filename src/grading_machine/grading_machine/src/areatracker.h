#pragma once

// std
#include <memory>

// ros

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
		
		typedef uint32_t tAreaId;
		
		/** this method is called in case of error to reset all areas
		*/
		virtual void clearAreas();
		
		/** this method is called to generate an id when a new area is detected
		*/
		virtual tAreaId getNewAreaId();
		
		/** this method is called when area has moved
		*/
		virtual void updateArea(tAreaId pId, int pIndexInFrame, const Frame & pFrame);
		
		/** this method is called to release id of an area that leave the frame
		*/
		virtual void releaseArea(tAreaId pId);
		
	};
	
	
	
	void clear();
	void setTrackingFunctor(AreaTrackingFunctor * pFunctor);
	void addNewFrame(const Frame & pFrame);
	
	
private:
	typedef std::vector<int> tIntVector;
	tIntVector 	mOffsets;

	AreaTrackingFunctor * mFunctor;
	tAreas		mPreviousAreas;
	tIntVector	mIds;
};

#pragma once

// std

// ros


#include "areatracker.h"


class DetectionModel;
class Frame;
namespace cv
{
	class Mat;
}

/** 
*/
class DetectionManager : public AreaTracker::AreaTrackingFunctor
{
public:
	DetectionManager();
	virtual ~DetectionManager();
	
	
	virtual void clearAreas();
	virtual tAreaId getNewAreaId();
	virtual void updateArea(tAreaId pId, int pIndexInFrame, const Frame & pFrame);
	virtual void releaseArea(tAreaId pId);
	
	void computeDebugFrame(cv::Mat & pMat, const Frame & pLastFrame);
private:
	struct TrackingContext
	{
		TrackingContext();
		~TrackingContext();
		
		/** model build from tracking of this area
		*/
		DetectionModel *	mModel;
		int					mIndexInLastFrame;
	};
	
	typedef std::vector< std::unique_ptr<TrackingContext> > tTrackingContexes;

	tTrackingContexes	mTracking;
};
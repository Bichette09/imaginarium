#include "detectionmanager.h"

// opencv
#include "opencv2/opencv.hpp"

// ros
#include <ros/ros.h>

#include "detectionmodel.h"

DetectionManager::TrackingContext::TrackingContext()
	: mModel(new DetectionModel())
	, mIndexInLastFrame(-1)
{
	
}

DetectionManager::TrackingContext::~TrackingContext()
{
	delete mModel; mModel = NULL;
}


DetectionManager::DetectionManager()
{
}

DetectionManager::~DetectionManager()
{
}

void DetectionManager::clearAreas()
{
	mTracking.clear();
}

DetectionManager::tAreaId DetectionManager::getNewAreaId()
{
	tAreaId lRes = mTracking.size();
	tTrackingContexes::iterator lIt = mTracking.begin();
	const tTrackingContexes::iterator lItEnd = mTracking.end();
	for( ; lIt != lItEnd ; ++lIt)
	{
		if(!lIt->get())
		{
			lRes = lIt - mTracking.begin();
			break;
		}
	}
	if(lRes >= mTracking.size())
	{
		lRes = mTracking.size();
		mTracking.push_back(tTrackingContexes::value_type());
	}
	
	tTrackingContexes::value_type lCtx(new TrackingContext());
	
	mTracking[lRes].swap(lCtx);
	return lRes;
}

void DetectionManager::updateArea(tAreaId pId, int pIndexInFrame, const Frame & pFrame)
{
	tTrackingContexes::value_type & lCtx = mTracking[pId];
	if(!lCtx)
	{
		ROS_ERROR_STREAM("updateArea Tracking context "<<pId<<" is missing");
		return;
	}
	lCtx->mModel->addAreaToModel(pFrame.getAreas()[pIndexInFrame]);
	lCtx->mIndexInLastFrame = pIndexInFrame;
}

void DetectionManager::releaseArea(tAreaId pId)
{
	tTrackingContexes::value_type lCtx;
	lCtx.swap(mTracking[pId]);
	if(!lCtx)
	{
		ROS_ERROR_STREAM("releaseArea Tracking context "<<pId<<" is missing");
		return;
	}
	
	
	std::cout<<"Area "<<pId<<" exit "<<lCtx->mModel->toString()<<std::endl;
	
}

void DetectionManager::computeDebugFrame(cv::Mat & pMat, const Frame & pLastFrame)
{
	tTrackingContexes::iterator lIt = mTracking.begin();
	const tTrackingContexes::iterator lItEnd = mTracking.end();
	for( ; lIt != lItEnd ; ++lIt)
	{
		if(!*lIt)
			continue;
		const AreaOfInterest & lArea = pLastFrame.getAreas()[(*lIt)->mIndexInLastFrame];
		// cv::rectangle(lYDownsized,lArea.mAABBMin,lArea.mAABBMax,255);
		
		// rotated rectangle
		cv::Point2f rect_points[4]; 
		lArea.mOBB.points( rect_points );
		for( int j = 0; j < 4; j++ )
			cv::line( pMat, rect_points[j], rect_points[(j+1)%4], 220, 1, 8 );
		if((!lArea.mIsAloneOnXAxis) || lArea.mOverlapBorder)
		{
			cv::line( pMat, rect_points[0], rect_points[2], 220, 1, 8 );
			cv::line( pMat, rect_points[1], rect_points[3], 220, 1, 8 );
		}
		
		std::stringstream lIdStr;
		lIdStr<<(lIt-mTracking.begin());
		cv::putText(pMat,lIdStr.str().c_str(),rect_points[0], cv::FONT_HERSHEY_SIMPLEX, 4,(0,0,255),2);
		
	}
}
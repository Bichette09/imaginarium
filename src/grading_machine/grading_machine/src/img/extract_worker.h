#pragma once

#include "frame.h"
#include "image_common/frame_processor.h"

class FilterWorker;

class ExtractWorker : public WorkerInterface<Frame>
{
public:
	
	struct Parameters
	{
		Parameters();
		
		int		mMinimumPixelsPerGroup;
		int		mMaxAreaToExtract;
		float	mMinimumBackgroundPercent;
		float	mMinimumSpaceBetweenAreaPercent;
		bool	mConnectivityFullWay;
	};
	
	ExtractWorker(FilterWorker & pFilterWorker, const Parameters & pParameters);
	virtual ~ExtractWorker();
	
protected:
	virtual bool computeNextResult(Frame & pRes);
private:
	FilterWorker & 				mFilterWorker;
	FrameProcessor<Frame> *		mFilterThread;
	const Parameters &			mParameters;
	
	cv::Mat mLabels;
	cv::Mat mStats;
	cv::Mat mCentroids;
	cv::Mat mNonZeroCoordinates;
};
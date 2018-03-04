#pragma once

#include "frame_processor.h"

class FilterWorker;

class ExtractWorker : public FrameProcessor::WorkerInterface
{
public:
	
	struct Parameters
	{
		Parameters();
		
		int		mMinimumPixelsPerGroup;
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
	FrameProcessor *			mFilterThread;
	const Parameters &			mParameters;
	
	cv::Mat mLabels;
	cv::Mat mStats;
	cv::Mat mCentroids;
	cv::Mat mNonZeroCoordinates;
};
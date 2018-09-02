#pragma once

#include "frame.h"
#include "image_common/frame_processor.h"
#include "image_common/frame_provider_worker.h"

class FilterThreadCompanion;

class FilterWorker : public WorkerInterface<Frame>
{
public:

	struct ChannelParameters
	{
		ChannelParameters();
		
		int		mThreshold;
		bool	mGaussian;
		bool 	mDilate;
	};
	
	struct Parameters
	{
		Parameters();
		
		ChannelParameters	mYParam;
		ChannelParameters	mUParam;
		ChannelParameters	mVParam;
		
		float	mExclusionZoneTopPercent;
		float	mExclusionZoneBottomPercent;
		bool	mErode;
	};
	
	FilterWorker(FrameProvider & pFrameProvider, const Parameters & pParameters);
	virtual ~FilterWorker();
	
	const Parameters &			mParameters;
	
protected:
	virtual bool computeNextResult(Frame & pRes);
private:
	FrameProviderWorker<Frame>	mFrameProviderWorker;
	FrameProcessor<Frame> *		mCameraThread;
	FilterThreadCompanion *		mUCompanion;
	FilterThreadCompanion *		mVCompanion;
	
	cv::Mat						mMorphoKernel5x5;
	cv::Mat						mTmpA;
	cv::Mat						mTmpB;
	cv::Mat						mTmpC;
};
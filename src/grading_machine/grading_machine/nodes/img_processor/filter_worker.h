#pragma once

#include "frame_processor.h"

class FilterThreadCompanion;
class CameraWorker;
class FilterWorker : public FrameProcessor::WorkerInterface
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
	
	FilterWorker(CameraWorker & pCameraWorker, const Parameters & pParameters);
	virtual ~FilterWorker();
	
	const Parameters &			mParameters;
	
protected:
	virtual bool computeNextResult(Frame & pRes);
private:
	CameraWorker & 				mCameraWorker;
	FrameProcessor *			mCameraThread;
	FilterThreadCompanion *		mUCompanion;
	FilterThreadCompanion *		mVCompanion;
	
	cv::Mat						mMorphoKernel5x5;
	cv::Mat						mTmpA;
	cv::Mat						mTmpB;
	cv::Mat						mTmpC;
};
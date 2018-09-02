#pragma once

#include "light_and_line_frame.h"
#include "image_common/frame_processor.h"
#include "image_common/frame_provider_worker.h"

class ThresholdingWorker : public WorkerInterface<LightAndLineFrame>
{
public:

	struct ColorAreaDefinition
	{
		ColorAreaDefinition();
		
		void setValuesFromString(const std::string & pString);
		std::string getStringFromValues() const;
		
		int	mUMin;
		int	mUMax;
		int	mVMin;
		int mVMax;
		int mYMin;
		float mAspectRatioMin;
		float mAspectRatioMax;
		int mMinimumPixelCount;
	};


	struct Parameters
	{
		Parameters();
		
		ColorAreaDefinition		mRedLightParameter;
		ColorAreaDefinition		mYellowLightParameter;
		ColorAreaDefinition		mBlueLightParameter;
	};
	
	ThresholdingWorker(FrameProvider & pFrameProvider, const Parameters & pParameters);
	virtual ~ThresholdingWorker();
	
	const Parameters &			mParameters;
	
protected:
	virtual bool computeNextResult(LightAndLineFrame & pRes);
private:

	void extractColorAreas(LightAndLineFrame & pFrame,const ColorAreaDefinition & pColorDef);

	FrameProviderWorker<LightAndLineFrame>	mFrameProviderWorker;
	FrameProcessor<LightAndLineFrame> *		mCameraThread;
	
	cv::Mat mTmpA,mTmpB,mTmpC,mTmpD;
	cv::Mat mLabels,mCentroids,mStats;
	cv::Mat mMorphoKernel;
};
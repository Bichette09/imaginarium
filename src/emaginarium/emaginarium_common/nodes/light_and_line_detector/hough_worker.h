#pragma once

#include "light_and_line_frame.h"
#include "image_common/frame_processor.h"
#include "image_common/frame_provider_worker.h"
#include "thresholding_worker.h"

class HoughWorker : public WorkerInterface<LightAndLineFrame>
{
public:

	struct LineDefinition
	{
		LineDefinition();
		
		void setValuesFromString(const std::string & pString);
		std::string getStringFromValues() const;
		
		ThresholdingWorker::ColorFilterParameter	mColorFilter;
		int						mHoughThreshold;
		int						mMinLineLen;
		int						mMaxLineGap;
		
	};
	
	struct Parameters
	{
		Parameters();
		
		LineDefinition				mRedLineColorParameter;
		LineDefinition				mGreenLineColorParameter;
	};
	
	HoughWorker(ThresholdingWorker & pThresholdingWorker, const Parameters & pParameters);
	virtual ~HoughWorker();
	
	const Parameters &			mParameters;
	
protected:
	virtual bool computeNextResult(LightAndLineFrame & pRes);
private:
	
	void extractLines(const LineDefinition & pLineDef, LightAndLineFrame & pRes,LightAndLineFrame::tLines & pLines);

	ThresholdingWorker &				mThresholdingWorker;
	FrameProcessor<LightAndLineFrame> *	mThresholdingThread;
	
	enum TmpMat
	{
		TM_A = 0,
		TM_B,
		TM_C,
		
		TM_a,
		
		TM_Count
	};
	
	cv::Mat mLineTmpMatArray[TM_Count];
	
	cv::Mat mMorphoKernel3x3;
	cv::Mat mMorphoKernel5x5;
	
};
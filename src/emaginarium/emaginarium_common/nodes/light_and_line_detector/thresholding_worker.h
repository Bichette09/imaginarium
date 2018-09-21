#pragma once

#include "light_and_line_frame.h"
#include "image_common/frame_processor.h"
#include "image_common/frame_provider_worker.h"

class ThresholdingWorker : public WorkerInterface<LightAndLineFrame>
{
public:

	struct ColorFilterParameter
	{
		ColorFilterParameter();
		
		int	mUMin;
		int	mUMax;
		int	mVMin;
		int mVMax;
		int mYMin;
		int mYMax;
		int mDebugMask;
	};

	struct LightColorAreaDefinition
	{
		LightColorAreaDefinition();
		
		void setValuesFromString(const std::string & pString);
		std::string getStringFromValues() const;
		
		ColorFilterParameter mColorFilter;
		
		/** downscale factor used to remove glitched detection after thresholding
		*/
		int mDownscaleFactor;
		/** how many percent of pixels should be valid per downscale area, eg for a downscale of 4, there is 16px per area
		*/
		int mPercentOfValidPixelPerArea;
	};
	
	
	
	struct SearchArea
	{
		SearchArea();
		
		void setValuesFromString(const std::string & pString);
		std::string getStringFromValues() const;
		cv::Rect getRoiRect(const cv::Mat & pImg) const;
		
		int						mXMinPercent;
		int						mXMaxPercent;
		int						mYMinPercent;
		int						mYMaxPercent;
		
	};

	struct Parameters
	{
		Parameters();
		
		SearchArea					mLightSearchArea;
		LightColorAreaDefinition	mRedLightParameter;
		LightColorAreaDefinition	mYellowLightParameter;
		LightColorAreaDefinition	mBlueLightParameter;
		
		SearchArea					mLineSearchArea;
		
		int32_t						mCannyThreshold;
	};
	
	ThresholdingWorker(FrameProvider & pFrameProvider, const Parameters & pParameters, const bool & pEnableLightDetection);
	virtual ~ThresholdingWorker();
	
	const Parameters &			mParameters;
	
	static void ComputeColorMask(LightAndLineFrame & pFrame,const cv::Rect & pLightSearchRoi,const ColorFilterParameter & pColorDef, cv::Mat * pTmpMatArray);
	
	
protected:
	virtual bool computeNextResult(LightAndLineFrame & pRes);
private:

	void extractColorAreas(LightAndLineFrame & pFrame,const cv::Rect & pLightSearchRoi, const LightColorAreaDefinition & pColorDef, cv::Mat * pTmpMatArray, LightAndLineFrame::tRects & pAreas);

	
	const bool & mEnableLightDetection;
	
	FrameProviderWorker<LightAndLineFrame>	mFrameProviderWorker;
	FrameProcessor<LightAndLineFrame> *		mCameraThread;
	
	enum TmpMat
	{
		TM_A = 0,
		TM_B,
		TM_C,
		
		TM_a,
		TM_b,
		
		TM_Labels,
		TM_Stats,
		TM_Centroids,
		
		TM_Count
	};
	
	cv::Mat mLightTmpMatArray[TM_Count];
	
	cv::Mat mMorphoKernel3x3;
	cv::Mat mMorphoKernel5x5;
	
};
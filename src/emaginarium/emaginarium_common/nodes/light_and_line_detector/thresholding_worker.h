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
		/** downscale factor used to remove glitched detection after thresholding
		*/
		int mDownscaleFactor;
		/** how many percent of pixels should be valid per downscale area, eg for a downscale of 4, there is 16px per area
		*/
		int mPercentOfValidPixelPerArea;
	};


	struct Parameters
	{
		Parameters();
		
		void setLightSearchArea(const std::string & pString);
		std::string getLightSearchArea() const;
		
		int						mLightSearchAreaXMinPercent;
		int						mLightSearchAreaXMaxPercent;
		int						mLightSearchAreaYMinPercent;
		int						mLightSearchAreaYMaxPercent;
		bool					mOutputLightDetectionDebugInfo;
		
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

	typedef std::vector<cv::Rect> tRects;
	void extractColorAreas(LightAndLineFrame & pFrame,const cv::Rect & pLightSearchRoi, const ColorAreaDefinition & pColorDef, tRects & pAreas);

	FrameProviderWorker<LightAndLineFrame>	mFrameProviderWorker;
	FrameProcessor<LightAndLineFrame> *		mCameraThread;
	
	cv::Mat mTmpA,mTmpB,mTmpC,mTmpD;
	cv::Mat mLabels,mCentroids,mStats;
	cv::Mat mMorphoKernel;
};
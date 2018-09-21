#include "thresholding_worker.h"

// ros
#include "ros/ros.h"



ThresholdingWorker::ColorFilterParameter::ColorFilterParameter()
	: mUMin(0)
	, mUMax(255)
	, mVMin(0)
	, mVMax(255)
	, mYMin(170)
	, mYMax(255)
	, mDebugMask(0)
{
}

ThresholdingWorker::LightColorAreaDefinition::LightColorAreaDefinition()
	: mDownscaleFactor(4)
	, mPercentOfValidPixelPerArea(33)
{
	
}

void ThresholdingWorker::LightColorAreaDefinition::setValuesFromString(const std::string & pString)
{
	LightColorAreaDefinition lBackup = *this;
	std::stringstream lStream(pString);
	lStream
		>>mColorFilter.mYMin
		>>mColorFilter.mYMax
		>>mColorFilter.mUMin
		>>mColorFilter.mUMax
		>>mColorFilter.mVMin
		>>mColorFilter.mVMax;
	while(lStream)
	{
		char lTmp = ' ';
		lStream>>lTmp;
		if(lTmp == '|')
			break;
	}
	lStream
		>>mColorFilter.mDebugMask;
	while(lStream)
	{
		char lTmp = ' ';
		lStream>>lTmp;
		if(lTmp == '|')
			break;
	}
	lStream
		>>mDownscaleFactor
		>>mPercentOfValidPixelPerArea;
	if(!lStream)
	{
		ROS_WARN_STREAM("Invalid light parameters "<<pString);
		*this = lBackup;
	}
}

std::string ThresholdingWorker::LightColorAreaDefinition::getStringFromValues() const
{
	std::stringstream lStream;
	lStream
		<<mColorFilter.mYMin<<" "
		<<mColorFilter.mYMax<<" "
		<<mColorFilter.mUMin<<" "
		<<mColorFilter.mUMax<<" "
		<<mColorFilter.mVMin<<" "
		<<mColorFilter.mVMax<<" "
		<<"| "
		<<mColorFilter.mDebugMask<<" "
		<<"| "
		<<mDownscaleFactor<<" "
		<<mPercentOfValidPixelPerArea<<" ";
	return lStream.str();
}




ThresholdingWorker::SearchArea::SearchArea()
	: mXMinPercent(0)
	, mXMaxPercent(0)
	, mYMinPercent(0)
	, mYMaxPercent(0)
{
}


void ThresholdingWorker::SearchArea::setValuesFromString(const std::string & pString)
{
	int lVals[4];
	std::stringstream lStream(pString);
	lStream
		>>lVals[0]
		>>lVals[1]
		>>lVals[2]
		>>lVals[3];
		
	if(!lStream)
	{
		ROS_WARN_STREAM("Invalid parameters "<<pString);
	}
	else
	{
		mXMinPercent = lVals[0];
		mXMaxPercent = lVals[1];
		mYMinPercent = lVals[2];
		mYMaxPercent = lVals[3];
	}
}

std::string ThresholdingWorker::SearchArea::getStringFromValues() const
{
	std::stringstream lStream;
	lStream
		<< mXMinPercent <<" "
		<< mXMaxPercent <<" "
		<< mYMinPercent <<" "
		<< mYMaxPercent <<" "
		;
	return lStream.str();
}

cv::Rect ThresholdingWorker::SearchArea::getRoiRect(const cv::Mat & pImg) const
{
	return cv::Rect( 
		cv::Point(
			pImg.cols * mXMinPercent / 100,
			pImg.rows * mYMinPercent / 100),
		cv::Point(
			pImg.cols * mXMaxPercent / 100,
			pImg.rows * mYMaxPercent / 100));
}

ThresholdingWorker::Parameters::Parameters()
	: mCannyThreshold(64)
{
	mLightSearchArea.setValuesFromString("50 100 10 90");
	mRedLightParameter.setValuesFromString("100 255 100 120 160 210 | 0 | 6 20");
	mYellowLightParameter.setValuesFromString("120 255 50 70 160 185 | 0 | 6 20");
	mBlueLightParameter.setValuesFromString("60 255 170 210 110 140 | 0 | 6 20");
	
	mLineSearchArea.setValuesFromString("0 100 70 95");
}


ThresholdingWorker::ThresholdingWorker(FrameProvider & pFrameProvider, const Parameters & pParameters, const bool & pEnableLightDetection)
	: mCameraThread(NULL)
	, mFrameProviderWorker(pFrameProvider)
	, mParameters(pParameters)
	, mEnableLightDetection(pEnableLightDetection)
{
	mCameraThread = new FrameProcessor<LightAndLineFrame>(mFrameProviderWorker);
}

ThresholdingWorker::~ThresholdingWorker()
{
	delete mCameraThread; mCameraThread = NULL;
}

bool ThresholdingWorker::computeNextResult(LightAndLineFrame & pRes)
{
	if(!mCameraThread->getNextFrame(pRes))
		return false;
	
	const cv::Mat lY =  pRes[LightAndLineFrame::Y];
	{
		pRes.setTimestamp(LightAndLineFrame::F_LightThresholdingStart);
		
		const cv::Rect lLightRoi = mParameters.mLightSearchArea.getRoiRect(lY);
		pRes.editLightSearchArea() = lLightRoi;
		
		extractColorAreas(pRes,lLightRoi,mParameters.mRedLightParameter,mLightTmpMatArray,pRes[LightAndLineFrame::LC_Red]);
		extractColorAreas(pRes,lLightRoi, mParameters.mYellowLightParameter,mLightTmpMatArray,pRes[LightAndLineFrame::LC_Yellow]);
		extractColorAreas(pRes,lLightRoi, mParameters.mBlueLightParameter,mLightTmpMatArray,pRes[LightAndLineFrame::LC_Blue]);

		pRes.setTimestamp(LightAndLineFrame::F_LightThresholdingDone);
	}
	
	{
		const cv::Rect lLineRoi = mParameters.mLineSearchArea.getRoiRect(lY);
		pRes.editLineSearchArea() = lLineRoi;
		
		pRes.setTimestamp(LightAndLineFrame::F_LineCanyFilterStart);
		
		// compute edges with a canny filter
		const cv::Mat lSubY(pRes.editY(),lLineRoi);
		cv::Canny(lSubY,pRes[LightAndLineFrame::CannyEdges],mParameters.mCannyThreshold,mParameters.mCannyThreshold*3,3);
		
		pRes.setTimestamp(LightAndLineFrame::F_LineCanyFilterDone);
		
		
	}
	
	
	return true;
}

void ThresholdingWorker::extractColorAreas(LightAndLineFrame & pFrame,const cv::Rect & pLightSearchRoi,const LightColorAreaDefinition & pColorDef, cv::Mat * pTmpMatArray, LightAndLineFrame::tRects & pAreas)
{
	pAreas.clear();
	if(!mEnableLightDetection)
	{
		return;
	}
	
	ComputeColorMask(pFrame,pLightSearchRoi,pColorDef.mColorFilter,pTmpMatArray);
	// output is in TM_A
	
	// we want to remove bad detections, we are looking for a big spot
	// to avoid using big and slow kernels for CLOSE/OPEN
	// we use a small hack by downsizing the image with INTER_AREA, which will compute average
	// then we use a new threshold to keep only areas where they were enought white pixels
	// then nice side effect connectedComponentsWithStats will run on a smaller image
	const float lInvDownScaleFactor = 1. / pColorDef.mDownscaleFactor;
	cv::resize(pTmpMatArray[TM_A],pTmpMatArray[TM_b],cv::Size(),lInvDownScaleFactor,lInvDownScaleFactor,cv::INTER_AREA);
	const int lThreshold = (255 * pColorDef.mPercentOfValidPixelPerArea) / 100;
	cv::threshold(pTmpMatArray[TM_b], pTmpMatArray[TM_a],lThreshold,255,cv::THRESH_BINARY);

	// mTmpA contains final color mask
	int lGroupCount = cv::connectedComponentsWithStats (
			pTmpMatArray[TM_a],
			pTmpMatArray[TM_Labels],
			pTmpMatArray[TM_Stats],
			pTmpMatArray[TM_Centroids],
			8,
			CV_16U,
			cv::CCL_DEFAULT);
	if(lGroupCount <= 10)
	{
		for(int i = 1 ; i < lGroupCount ; ++i)
		{
			pAreas.push_back( cv::Rect(
				pLightSearchRoi.x + pTmpMatArray[TM_Stats].at<int32_t>(i,cv::CC_STAT_LEFT) * pColorDef.mDownscaleFactor,
				pLightSearchRoi.y + pTmpMatArray[TM_Stats].at<int32_t>(i,cv::CC_STAT_TOP) * pColorDef.mDownscaleFactor,
				pTmpMatArray[TM_Stats].at<int32_t>(i,cv::CC_STAT_WIDTH)* pColorDef.mDownscaleFactor,
				pTmpMatArray[TM_Stats].at<int32_t>(i,cv::CC_STAT_HEIGHT)* pColorDef.mDownscaleFactor));
		}
	}
	
}

void ThresholdingWorker::ComputeColorMask(LightAndLineFrame & pFrame,const cv::Rect & pLightSearchRoi, const ColorFilterParameter & pColorDef, cv::Mat * pTmpMatArray)
{
	const cv::Mat lU(pFrame.editU(),pLightSearchRoi);
	const cv::Mat lV(pFrame.editV(),pLightSearchRoi);
	const cv::Mat lY(pFrame.editY(),pLightSearchRoi);
	
	cv::Mat const * const lSrc[6] = {&lY,&lY,&lU,&lU,&lV,&lV};
	int const lThValue[6] = {pColorDef.mYMin,pColorDef.mYMax,pColorDef.mUMin,pColorDef.mUMax,pColorDef.mVMin,pColorDef.mVMax};
	int const lThType[6] = {cv::THRESH_BINARY,cv::THRESH_BINARY_INV,cv::THRESH_BINARY,cv::THRESH_BINARY_INV,cv::THRESH_BINARY,cv::THRESH_BINARY_INV};
	
	bool lIsFirstMask = true;
	TmpMat lAccumulationMat = TM_A;
	TmpMat lNextAccumulationMat = TM_B;
	for(int i = 0 ; i < 6 ; ++i)
	{
		if( (lThType[i] == cv::THRESH_BINARY && lThValue[i] <= 0) ||
			(lThType[i] == cv::THRESH_BINARY_INV && lThValue[i] >= 255))
		{
			continue;
		}
		cv::threshold(*lSrc[i], pTmpMatArray[lIsFirstMask ? lAccumulationMat : TM_C],lThValue[i],255,lThType[i]);
		
		if(lIsFirstMask)
		{
			lIsFirstMask = false;
		}
		else
		{
			cv::bitwise_and(pTmpMatArray[lAccumulationMat],pTmpMatArray[TM_C],pTmpMatArray[lNextAccumulationMat]);
			std::swap(lAccumulationMat,lNextAccumulationMat);
		}
	}
	
	if(lIsFirstMask)
	{
		cv::threshold(lY,pTmpMatArray[TM_A],255,255,cv::THRESH_BINARY_INV);
	}
	else if(lAccumulationMat != TM_A)
	{
		cv::swap(pTmpMatArray[lAccumulationMat],pTmpMatArray[lNextAccumulationMat]);
	}
	
	if(pColorDef.mDebugMask)
	{
		pFrame[LightAndLineFrame::Debug] = pTmpMatArray[TM_A].clone();
	}
	
}
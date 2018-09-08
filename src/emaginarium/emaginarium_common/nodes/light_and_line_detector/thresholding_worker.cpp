#include "thresholding_worker.h"

// ros
#include "ros/ros.h"


ThresholdingWorker::ColorAreaDefinition::ColorAreaDefinition()
	: mUMin(0)
	, mUMax(255)
	, mVMin(0)
	, mVMax(255)
	, mYMin(170)
	, mYMax(255)
	, mDownscaleFactor(4)
	, mPercentOfValidPixelPerArea(33)
{
	
}

void ThresholdingWorker::ColorAreaDefinition::setValuesFromString(const std::string & pString)
{
	ColorAreaDefinition lBackup = *this;
	std::stringstream lStream(pString);
	lStream
		>>mYMin
		>>mYMax
		>>mUMin
		>>mUMax
		>>mVMin
		>>mVMax
		>>mDownscaleFactor
		>>mPercentOfValidPixelPerArea;
	if(!lStream)
	{
		ROS_WARN_STREAM("Invalid parameters "<<pString);
		*this = lBackup;
	}
}

std::string ThresholdingWorker::ColorAreaDefinition::getStringFromValues() const
{
	std::stringstream lStream;
	lStream
		<<mYMin<<" "
		<<mYMax<<" "
		<<mUMin<<" "
		<<mUMax<<" "
		<<mVMin<<" "
		<<mVMax<<" "
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
{
	mLightSearchArea.setValuesFromString("50 100 10 90");
	mRedLightParameter.setValuesFromString("100 255 100 120 160 210 6 20");
	mYellowLightParameter.setValuesFromString("120 255 50 70 160 185 6 20");
	mBlueLightParameter.setValuesFromString("60 255 170 210 110 140 6 20");
	
	mLineSearchArea.setValuesFromString("0 100 70 95");
	mLineColorParameter.setValuesFromString("0 255 0 255 0 255 0 0");
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
	pRes.setTimestamp(LightAndLineFrame::F_LightThresholdingStart);
	
	const cv::Mat lY =  pRes[LightAndLineFrame::Y];
	const cv::Rect lLightRoi = mParameters.mLightSearchArea.getRoiRect(lY);
	
	pRes.editLightSearchArea() = lLightRoi;
	extractColorAreas(pRes,lLightRoi,mParameters.mRedLightParameter,mLightTmpMatArray,pRes[LightAndLineFrame::LC_Red]);
	extractColorAreas(pRes,lLightRoi, mParameters.mYellowLightParameter,mLightTmpMatArray,pRes[LightAndLineFrame::LC_Yellow]);
	extractColorAreas(pRes,lLightRoi, mParameters.mBlueLightParameter,mLightTmpMatArray,pRes[LightAndLineFrame::LC_Blue]);

	pRes.setTimestamp(LightAndLineFrame::F_LightThresholdingDone);
	
	return true;
}

void ThresholdingWorker::extractColorAreas(LightAndLineFrame & pFrame,const cv::Rect & pLightSearchRoi,const ColorAreaDefinition & pColorDef, cv::Mat * pTmpMatArray, LightAndLineFrame::tRects & pAreas)
{
	pAreas.clear();
	if(!mEnableLightDetection)
		return;
	
	computeColorMask(pFrame,pLightSearchRoi,pColorDef,pTmpMatArray);
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

void ThresholdingWorker::computeColorMask(LightAndLineFrame & pFrame,const cv::Rect & pLightSearchRoi, const ColorAreaDefinition & pColorDef, cv::Mat * pTmpMatArray)
{
	
	const cv::Mat lU(pFrame.editU(),pLightSearchRoi);
	cv::threshold(lU, pTmpMatArray[TM_C],pColorDef.mUMin,255,cv::THRESH_BINARY);
	cv::threshold(lU, pTmpMatArray[TM_D],pColorDef.mUMax,255,cv::THRESH_BINARY_INV);
	cv::bitwise_and(pTmpMatArray[TM_C],pTmpMatArray[TM_D],pTmpMatArray[TM_A]);
	// TM_A contains U mask
	
	const cv::Mat lV(pFrame.editV(),pLightSearchRoi);
	cv::threshold(lV, pTmpMatArray[TM_C],pColorDef.mVMin,255,cv::THRESH_BINARY);
	cv::threshold(lV, pTmpMatArray[TM_D],pColorDef.mVMax,255,cv::THRESH_BINARY_INV);
	cv::bitwise_and(pTmpMatArray[TM_C],pTmpMatArray[TM_A],pTmpMatArray[TM_B]);
	cv::bitwise_and(pTmpMatArray[TM_D],pTmpMatArray[TM_B],pTmpMatArray[TM_A]);
	// TM_A contains U & V mask
	
	const cv::Mat lY(pFrame.editV(),pLightSearchRoi);
	cv::threshold(lY, pTmpMatArray[TM_C],pColorDef.mYMin,255,cv::THRESH_BINARY);
	cv::threshold(lY, pTmpMatArray[TM_D],pColorDef.mYMax,255,cv::THRESH_BINARY_INV);
	cv::bitwise_and(pTmpMatArray[TM_C],pTmpMatArray[TM_A],pTmpMatArray[TM_B]);
	cv::bitwise_and(pTmpMatArray[TM_D],pTmpMatArray[TM_B],pTmpMatArray[TM_A]);
	// TM_A contains U & V & Y mask
}
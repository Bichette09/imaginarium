#include "thresholding_worker.h"

// ros
#include "ros/ros.h"


ThresholdingWorker::ColorAreaDefinition::ColorAreaDefinition()
	: mUMin(0)
	, mUMax(255)
	, mVMin(0)
	, mVMax(255)
	, mYMin(170)
	, mDownscaleFactor(4)
	, mPercentOfValidPixelPerArea(33)
{
	
}

void ThresholdingWorker::ColorAreaDefinition::setValuesFromString(const std::string & pString)
{
	ColorAreaDefinition lBackup = *this;
	std::stringstream lStream(pString);
	lStream
		>>mUMin
		>>mUMax
		>>mVMin
		>>mVMax
		>>mYMin
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
		<<mUMin<<" "
		<<mUMax<<" "
		<<mVMin<<" "
		<<mVMax<<" "
		<<mYMin<<" "
		<<mDownscaleFactor<<" "
		<<mPercentOfValidPixelPerArea<<" ";
	return lStream.str();
}

ThresholdingWorker::Parameters::Parameters()
	: mLightSearchAreaXMinPercent(50)
	, mLightSearchAreaXMaxPercent(100)
	, mLightSearchAreaYMinPercent(10)
	, mLightSearchAreaYMaxPercent(90)
{
	
}

void ThresholdingWorker::Parameters::setLightSearchArea(const std::string & pString)
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
		mLightSearchAreaXMinPercent = lVals[0];
		mLightSearchAreaXMaxPercent = lVals[1];
		mLightSearchAreaYMinPercent = lVals[2];
		mLightSearchAreaYMaxPercent = lVals[3];
	}
}

std::string ThresholdingWorker::Parameters::getLightSearchArea() const
{
	std::stringstream lStream;
	lStream
		<< mLightSearchAreaXMinPercent <<" "
		<< mLightSearchAreaXMaxPercent <<" "
		<< mLightSearchAreaYMinPercent <<" "
		<< mLightSearchAreaYMaxPercent <<" "
		;
	return lStream.str();
}

ThresholdingWorker::ThresholdingWorker(FrameProvider & pFrameProvider, const Parameters & pParameters, const bool & pEnableLightDetection)
	: mCameraThread(NULL)
	, mFrameProviderWorker(pFrameProvider)
	, mParameters(pParameters)
	, mEnableLightDetection(pEnableLightDetection)
{
	mCameraThread = new FrameProcessor<LightAndLineFrame>(mFrameProviderWorker);
	mMorphoKernel = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(7,7));
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
	const cv::Rect lLightRoi( 
		cv::Point(
			lY.cols * mParameters.mLightSearchAreaXMinPercent / 100,
			lY.rows * mParameters.mLightSearchAreaYMinPercent / 100),
		cv::Point(
			lY.cols * mParameters.mLightSearchAreaXMaxPercent / 100,
			lY.rows * mParameters.mLightSearchAreaYMaxPercent / 100));
	pRes.editLightSearchArea() = lLightRoi;
	extractColorAreas(pRes,lLightRoi,mParameters.mRedLightParameter,pRes[LightAndLineFrame::LC_Red]);
	extractColorAreas(pRes,lLightRoi, mParameters.mYellowLightParameter,pRes[LightAndLineFrame::LC_Yellow]);
	extractColorAreas(pRes,lLightRoi, mParameters.mBlueLightParameter,pRes[LightAndLineFrame::LC_Blue]);

	pRes.setTimestamp(LightAndLineFrame::F_LightThresholdingDone);
	
	return true;
}

void ThresholdingWorker::extractColorAreas(LightAndLineFrame & pFrame,const cv::Rect & pLightSearchRoi,const ColorAreaDefinition & pColorDef, LightAndLineFrame::tRects & pAreas)
{
	pAreas.clear();
	if(!mEnableLightDetection)
		return;
	
	const cv::Mat lU(pFrame.editU(),pLightSearchRoi);
	cv::threshold(lU, mTmpA,pColorDef.mUMin,255,cv::THRESH_BINARY);
	cv::threshold(lU, mTmpB,pColorDef.mUMax,255,cv::THRESH_BINARY_INV);
	cv::bitwise_and(mTmpA,mTmpB,mTmpC);
	// mTmpC contains U mask
	
	const cv::Mat lV(pFrame.editV(),pLightSearchRoi);
	cv::threshold(lV, mTmpA,pColorDef.mVMin,255,cv::THRESH_BINARY);
	cv::threshold(lV, mTmpB,pColorDef.mVMax,255,cv::THRESH_BINARY_INV);
	cv::bitwise_and(mTmpA,mTmpB,mTmpD);
	// mTmpD contains V mask
	
	const cv::Mat lY(pFrame.editV(),pLightSearchRoi);
	cv::threshold(lY, mTmpA,pColorDef.mYMin,255,cv::THRESH_BINARY);
	// mTmpA contains Y mask
	
	cv::bitwise_and(mTmpA,mTmpC,mTmpB);
	cv::bitwise_and(mTmpB,mTmpD,mTmpA);
	// mTmpA contains color mask
	
	// we want to remove bad detections, we are looking for a big spot
	// to avoid using big and slow kernels for CLOSE/OPEN
	// we use a small hack by downsizing the image with INTER_AREA, which will compute average
	// then we use a new threshold to keep only areas where they were enought white pixels
	// then nice side effect connectedComponentsWithStats will run on a smaller image
	const float lInvDownScaleFactor = 1. / pColorDef.mDownscaleFactor;
	cv::resize(mTmpA,mTmpB,cv::Size(),lInvDownScaleFactor,lInvDownScaleFactor,cv::INTER_AREA);
	const int lThreshold = (255 * pColorDef.mPercentOfValidPixelPerArea) / 100;
	cv::threshold(mTmpB, mTmpA,lThreshold,255,cv::THRESH_BINARY);

	// mTmpA contains final color mask
	int lGroupCount = cv::connectedComponentsWithStats (
			mTmpA,
			mLabels,
			mStats,
			mCentroids,
			8,
			CV_16U,
			cv::CCL_DEFAULT);
	if(lGroupCount <= 10)
	{
		for(int i = 1 ; i < lGroupCount ; ++i)
		{
			pAreas.push_back( cv::Rect(
				pLightSearchRoi.x + mStats.at<int32_t>(i,cv::CC_STAT_LEFT) * pColorDef.mDownscaleFactor,
				pLightSearchRoi.y + mStats.at<int32_t>(i,cv::CC_STAT_TOP) * pColorDef.mDownscaleFactor,
				mStats.at<int32_t>(i,cv::CC_STAT_WIDTH)* pColorDef.mDownscaleFactor,
				mStats.at<int32_t>(i,cv::CC_STAT_HEIGHT)* pColorDef.mDownscaleFactor));
		}
	}
	
}
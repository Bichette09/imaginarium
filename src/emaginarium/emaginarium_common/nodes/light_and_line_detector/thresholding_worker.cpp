#include "thresholding_worker.h"

// ros
#include "ros/ros.h"


ThresholdingWorker::ColorAreaDefinition::ColorAreaDefinition()
	: mUMin(0)
	, mUMax(255)
	, mVMin(0)
	, mVMax(255)
	, mYMin(170)
	, mMinimumPixelCount(20)
	, mAspectRatioMin(0.)
	, mAspectRatioMax(1000.)
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
		>>mMinimumPixelCount
		>>mAspectRatioMin
		>>mAspectRatioMax;
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
		<<mMinimumPixelCount<<" "
		<<mAspectRatioMin<<" "
		<<mAspectRatioMax<<" ";
	return lStream.str();
}

ThresholdingWorker::Parameters::Parameters()
{
	
}

ThresholdingWorker::ThresholdingWorker(FrameProvider & pFrameProvider, const Parameters & pParameters)
	: mCameraThread(NULL)
	, mFrameProviderWorker(pFrameProvider)
	, mParameters(pParameters)
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
	pRes.setTimestamp(LightAndLineFrame::F_ThresholdingStart);
	
	pRes[LightAndLineFrame::Debug] = pRes[LightAndLineFrame::Y].clone();
	
	extractColorAreas(pRes,mParameters.mRedLightParameter);
	extractColorAreas(pRes,mParameters.mYellowLightParameter);
	extractColorAreas(pRes,mParameters.mBlueLightParameter);
	
	pRes.setTimestamp(LightAndLineFrame::F_ThresholdingDone);
	
	return true;
}

void ThresholdingWorker::extractColorAreas(LightAndLineFrame & pFrame,const ColorAreaDefinition & pColorDef)
{
	cv::threshold(pFrame.editU(), mTmpA,pColorDef.mUMin,255,cv::THRESH_BINARY);
	cv::threshold(pFrame.editU(), mTmpB,pColorDef.mUMax,255,cv::THRESH_BINARY_INV);
	cv::bitwise_and(mTmpA,mTmpB,mTmpC);
	// mTmpC contains U mask
	
	cv::threshold(pFrame.editV(), mTmpA,pColorDef.mVMin,255,cv::THRESH_BINARY);
	cv::threshold(pFrame.editV(), mTmpB,pColorDef.mVMax,255,cv::THRESH_BINARY_INV);
	cv::bitwise_and(mTmpA,mTmpB,mTmpD);
	// mTmpD contains V mask
	
	cv::threshold(pFrame.editY(), mTmpA,pColorDef.mYMin,255,cv::THRESH_BINARY);
	// mTmpA contains Y mask
	
	cv::bitwise_and(mTmpA,mTmpC,mTmpB);
	cv::bitwise_and(mTmpB,mTmpD,mTmpA);
	// mTmpA contains color mask
	
	const int lDownScaleFactor = 4;
	cv::resize(mTmpA,mTmpB,cv::Size(),1./lDownScaleFactor,1./lDownScaleFactor,cv::INTER_AREA);
	cv::threshold(mTmpB, mTmpA,50,255,cv::THRESH_BINARY);
	
	// remove small glitches
	// first close groups, then remove small ones
	//cv::morphologyEx(mTmpA,mTmpB,cv::MORPH_CLOSE,mMorphoKernel);
	//cv::morphologyEx(mTmpB,mTmpA,cv::MORPH_OPEN,mMorphoKernel);
	// mTmpA contains final color mask
	//pFrame[LightAndLineFrame::Debug] = mTmpA.clone();
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
			int lAABBMinX = mStats.at<int32_t>(i,cv::CC_STAT_LEFT) * lDownScaleFactor;
			int lAABBMinY = mStats.at<int32_t>(i,cv::CC_STAT_TOP) * lDownScaleFactor;
			int lAABBMaxX = lAABBMinX + mStats.at<int32_t>(i,cv::CC_STAT_WIDTH)* lDownScaleFactor;
			int lAABBMaxY = lAABBMinY + mStats.at<int32_t>(i,cv::CC_STAT_HEIGHT)* lDownScaleFactor;
			int lPixelCount = mStats.at<int32_t>(i,cv::CC_STAT_AREA)* lDownScaleFactor* lDownScaleFactor;
			
			if(lPixelCount < pColorDef.mMinimumPixelCount)
				continue;
			
			cv::rectangle(pFrame[LightAndLineFrame::Debug],cv::Point(lAABBMinX-4,lAABBMinY-4),cv::Point(lAABBMaxX+4,lAABBMaxY+4),255,1);
			cv::rectangle(pFrame[LightAndLineFrame::Debug],cv::Point(lAABBMinX-3,lAABBMinY-3),cv::Point(lAABBMaxX+3,lAABBMaxY+3),64,1);
			cv::rectangle(pFrame[LightAndLineFrame::Debug],cv::Point(lAABBMinX-2,lAABBMinY-2),cv::Point(lAABBMaxX+2,lAABBMaxY+2),255,1);
		}
	}
	
}
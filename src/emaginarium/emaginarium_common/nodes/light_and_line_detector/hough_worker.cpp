#include "hough_worker.h"

// ros
#include "ros/ros.h"



HoughWorker::LineDefinition::LineDefinition()
	: mHoughThreshold(20)
	, mMinLineLen(20)
	, mMaxLineGap(20)
{
	
}

void HoughWorker::LineDefinition::setValuesFromString(const std::string & pString)
{
	LineDefinition lBackup = *this;
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
		>>mHoughThreshold
		>>mMinLineLen
		>>mMaxLineGap
		;
	if(!lStream)
	{
		ROS_WARN_STREAM("Invalid line parameters "<<pString);
		*this = lBackup;
	}
}

std::string HoughWorker::LineDefinition::getStringFromValues() const
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
		<<mHoughThreshold<<" "
		<<mMinLineLen<<" "
		<<mMaxLineGap<<" "
		;
	return lStream.str();
}

HoughWorker::Parameters::Parameters()
{
	mRedLineColorParameter.setValuesFromString("0 255 0 255 0 255 | 0 | 100 20 20 20 ");
	mGreenLineColorParameter.setValuesFromString("0 255 0 255 0 255 | 0 | 100 20 20 20 ");
}


HoughWorker::HoughWorker(ThresholdingWorker & pThresholdingWorker, const Parameters & pParameters)
	: mThresholdingWorker(pThresholdingWorker)
	, mParameters(pParameters)
	, mThresholdingThread(NULL)
{
	mMorphoKernel3x3 = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(3,3));
	mMorphoKernel5x5 = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(5,5));
	mThresholdingThread = new FrameProcessor<LightAndLineFrame>(pThresholdingWorker);
}

HoughWorker::~HoughWorker()
{
	delete mThresholdingThread; mThresholdingThread = NULL;
}

bool HoughWorker::computeNextResult(LightAndLineFrame & pRes)
{
	if(!mThresholdingThread->getNextFrame(pRes))
		return false;
	
	pRes.setTimestamp(LightAndLineFrame::F_LineHoughFilterStart);
	extractLines(mParameters.mRedLineColorParameter,pRes,pRes.editRedLines());
	extractLines(mParameters.mGreenLineColorParameter,pRes,pRes.editGreenLines());
	pRes.setTimestamp(LightAndLineFrame::F_LineHoughFilterDone);
	
	return true;
}

void HoughWorker::extractLines(const LineDefinition & pLineDef, LightAndLineFrame & pRes,LightAndLineFrame::tLines & pLines)
{
	pLines.clear();
	const cv::Rect & lLineRoi = pRes.getLineSearchArea();

	// compute a mask based on line color
	ThresholdingWorker::ComputeColorMask(pRes,lLineRoi,pLineDef.mColorFilter,mLineTmpMatArray);
	
	// dilate this mask
	float lDilatePixel = 16.;
	cv::resize(mLineTmpMatArray[TM_A],mLineTmpMatArray[TM_a],cv::Size(),1/lDilatePixel,1/lDilatePixel,cv::INTER_AREA);
	cv::resize(mLineTmpMatArray[TM_a],mLineTmpMatArray[TM_A],mLineTmpMatArray[TM_A].size(),lDilatePixel,lDilatePixel,cv::INTER_AREA);
	cv::threshold(mLineTmpMatArray[TM_A], mLineTmpMatArray[TM_B],10,255,cv::THRESH_BINARY);

	// compose mask and edges
	cv::bitwise_and(pRes[LightAndLineFrame::CannyEdges],mLineTmpMatArray[TM_B],mLineTmpMatArray[TM_A]);
	
	//pRes[LightAndLineFrame::Debug] = mLineTmpMatArray[TM_A].clone();
	
	
	cv::HoughLinesP(mLineTmpMatArray[TM_A],pLines,1,2*CV_PI/180, pLineDef.mHoughThreshold,pLineDef.mMinLineLen,pLineDef.mMaxLineGap);
	LightAndLineFrame::tLines::iterator lIt = pLines.begin();
	const LightAndLineFrame::tLines::iterator lItEnd = pLines.end();
	for( ; lIt != lItEnd ; ++lIt)
	{
		cv::Vec4i & lLine = *lIt;
		lLine[0] += lLineRoi.x;
		lLine[1] += lLineRoi.y;
		lLine[2] += lLineRoi.x;
		lLine[3] += lLineRoi.y;
	}
	
}

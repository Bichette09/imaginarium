#include "filter_worker.h"

#include "camera_worker.h"

// ros
#include "ros/ros.h"

FilterWorker::ChannelParameters::ChannelParameters()
	: mThreshold(-1)
	, mGaussian(false)
	, mDilate(false)
{
}

FilterWorker::Parameters::Parameters()
	: mErode(false)
	, mExclusionZoneTopPercent(0.01)
	, mExclusionZoneBottomPercent(0.01)
	
{
}


void processChannel(const cv::Mat & pIn, const cv::Mat & pMorphoKernel, cv::Mat & pOut, cv::Mat & pTmp, const FilterWorker::ChannelParameters & pParameters, bool pInverse)
{
	cv::Mat const * lThIn = &pIn;
	if(pParameters.mGaussian)
	{
		cv::GaussianBlur(pIn,pTmp,cv::Size(5,5),0);
		lThIn = &pTmp;
	}
	cv::threshold(*lThIn,pOut,pParameters.mThreshold,255,
	(pInverse ? cv::THRESH_BINARY_INV : cv::THRESH_BINARY )+ 
	(pParameters.mThreshold < 0 ? cv::THRESH_OTSU : 0)
	);
	if(pParameters.mDilate)
	{
		// if mask is inverted we should do erode
		cv::morphologyEx(pOut,pTmp,( pInverse ? cv::MORPH_ERODE : cv::MORPH_DILATE),pMorphoKernel);
		cv::swap(pTmp,pOut);
	}
}

#define USE_THREAD

class FilterThreadCompanion : public ThreadInterface
{
public:
	FilterThreadCompanion(const FilterWorker::ChannelParameters & pParameters, const cv::Mat & pMorphoKernel)
		: mQuit(false)
		, mInput(NULL)
		, mParameters(pParameters)
		, mMorphoKernel(pMorphoKernel)
	{
#ifdef USE_THREAD
		startThread();
#endif 
	}
	
	virtual ~FilterThreadCompanion()
	{
#ifdef USE_THREAD
		{
			std::unique_lock<std::mutex> lLock(mMutex);
			mQuit = true;
			mWaitCondition.notify_all();
			mWaitConditionResult.notify_all();
		}
		waitThread();
#endif
	}
	
	void setChannelToFilter(const cv::Mat & pMat)
	{
#ifdef USE_THREAD
		std::unique_lock<std::mutex> lLock(mMutex);
		mInput = &pMat;
		mWaitCondition.notify_all();
#else
		mInput = &pMat;
#endif
	}
	
	void getResult(cv::Mat & pRes)
	{
#ifdef USE_THREAD
		std::unique_lock<std::mutex> lLock(mMutex);
		if(mQuit)
			return;
		if(mInput)
		{
			ROS_WARN_STREAM("Companion is not ready !!");
			// job was not take by the worker !!!
			mWaitConditionResult.wait(lLock);
		}
		cv::swap(pRes,mResult);
#else
		processChannel(*mInput,mMorphoKernel,pRes,mTmp,mParameters,true);

#endif
	}
	
protected:
	virtual void run()
	{
		std::unique_lock<std::mutex> lLock(mMutex);
		while(!mQuit)
		{
			if(!mInput)
			{
				mWaitCondition.wait(lLock);
				continue;
			}
			
			processChannel(*mInput,mMorphoKernel,mResult,mTmp,mParameters,true);

			mInput = NULL;
			mWaitConditionResult.notify_all();
		}
	}
private:
	const FilterWorker::ChannelParameters &		mParameters;
	const cv::Mat & 			mMorphoKernel;
	std::mutex					mMutex;
	std::condition_variable		mWaitCondition;
	std::condition_variable		mWaitConditionResult;
	cv::Mat const *				mInput;
	cv::Mat						mResult;
	cv::Mat 					mTmp;
	bool						mQuit;
};


FilterWorker::FilterWorker(CameraWorker & pCameraWorker, const Parameters & pParameters)
	: mCameraThread(NULL)
	, mCameraWorker(pCameraWorker)
	, mParameters(pParameters)
{
	mCameraThread = new FrameProcessor(mCameraWorker);
	
	mMorphoKernel5x5 = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(5,5));
	mTmpA = cv::Mat(mCameraWorker.mParameters.mHalfHeight,mCameraWorker.mParameters.mHalfWidth,CV_8UC1);
	mTmpB = cv::Mat(mCameraWorker.mParameters.mHalfHeight,mCameraWorker.mParameters.mHalfWidth,CV_8UC1);
	mTmpC = cv::Mat(mCameraWorker.mParameters.mHalfHeight,mCameraWorker.mParameters.mHalfWidth,CV_8UC1);
	
	mUCompanion = new FilterThreadCompanion(mParameters.mUParam,mMorphoKernel5x5);
	mVCompanion = new FilterThreadCompanion(mParameters.mVParam,mMorphoKernel5x5);
}

FilterWorker::~FilterWorker()
{
	delete mUCompanion; mUCompanion = NULL;
	delete mVCompanion; mVCompanion = NULL;
	delete mCameraThread; mCameraThread = NULL;
}

bool FilterWorker::computeNextResult(Frame & pRes)
{
	if(!mCameraThread->getNextFrame(pRes))
		return false;
	
	const int lWidth = pRes[Frame::BackgroundMask].cols;
	const int lHeight = pRes[Frame::BackgroundMask].rows;
	
	pRes.setTimestamp(Frame::F_FilterStart);
	mVCompanion->setChannelToFilter(pRes[Frame::V]);
	mUCompanion->setChannelToFilter(pRes[Frame::U]);
	processChannel(pRes[Frame::Y], mMorphoKernel5x5,mTmpC,pRes[Frame::BackgroundMask],mParameters.mYParam,false);
	mVCompanion->getResult(mTmpA);
	mUCompanion->getResult(mTmpB);
	cv::bitwise_and(mTmpB,mTmpA,pRes[Frame::BackgroundMask]);
	cv::bitwise_not(pRes[Frame::BackgroundMask],mTmpA);
	cv::bitwise_or(mTmpA,mTmpC,pRes[Frame::BackgroundMask]);
	if(mParameters.mErode)
	{
		cv::morphologyEx(pRes[Frame::BackgroundMask],mTmpA,cv::MORPH_ERODE,mMorphoKernel5x5);
		cv::swap(pRes[Frame::BackgroundMask],mTmpA);
	}
	
	// clear top and bottom (we capture border of camera support)
	if(mParameters.mExclusionZoneTopPercent)
	{
		const cv::Point lTopEdgeRectA = cv::Point(0,0);
		const cv::Point lTopEdgeRectB = cv::Point(lWidth,(int)(lHeight * mParameters.mExclusionZoneTopPercent));
		cv::rectangle(pRes[Frame::BackgroundMask],lTopEdgeRectA,lTopEdgeRectB,0,CV_FILLED);
	}
	if(mParameters.mExclusionZoneBottomPercent)
	{
		const cv::Point lBottomEdgeRectA = cv::Point(0,lHeight - 1 - (int)(lHeight * mParameters.mExclusionZoneBottomPercent));
		const cv::Point lBottomEdgeRectB = cv::Point(lWidth,lHeight - 1);
		cv::rectangle(pRes[Frame::BackgroundMask],lBottomEdgeRectA,lBottomEdgeRectB,0,CV_FILLED);
	}
	
	pRes.setTimestamp(Frame::F_FilterDone);
	
	return true;
}
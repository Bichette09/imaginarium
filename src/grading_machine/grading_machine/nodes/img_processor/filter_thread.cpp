#include "filter_thread.h"

FilterThread::ChannelParameters::ChannelParameters()
	: mThreshold(-1)
	, mGaussian(false)
	, mDilate(false)
{
}

FilterThread::Parameters::Parameters()
	: mErode(false)
	, mExclusionZoneTopPercent(0.01)
	, mExclusionZoneBottomPercent(0.01)
	
{
}

FilterThread::FilterThread(CameraThread & pCameraThread, const Parameters & pParameters)
	: mCameraThread(pCameraThread)
	, mQuit(false)
	, mGotNewFrame(false)
	, mParameters(pParameters)
{
	startThread();
}

FilterThread::~FilterThread()
{
	{
		std::unique_lock<std::mutex> lLock(mMutex);
		mQuit = true;
		mWaitCondition.notify_all();
	}
	waitThread();
}

bool FilterThread::getNextFrame(GrabbedFrame & pFrame,int pTimeoutMs)
{
	std::unique_lock<std::mutex> lLock(mMutex);
	if(!mGotNewFrame)
	{
		mWaitCondition.wait_until(lLock,std::chrono::system_clock::now() + std::chrono::milliseconds(pTimeoutMs));
	}
	if(!mGotNewFrame)
		return false;
	mFrame.swap(pFrame);
	mGotNewFrame = false;
	mWaitCondition.notify_all();
	return true;
}

void processChannel(const cv::Mat & pIn, const cv::Mat & pMorphoKernel, cv::Mat & pOut, cv::Mat & pTmp, const FilterThread::ChannelParameters & pParameters, bool pInverse)
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

class FilterThreadCompanion : public ThreadInterface
{
public:
	FilterThreadCompanion(const FilterThread::ChannelParameters & pParameters, const cv::Mat & pMorphoKernel)
		: mQuit(false)
		, mInput(NULL)
		, mParameters(pParameters)
		, mMorphoKernel(pMorphoKernel)
	{
		startThread();
	}
	
	virtual ~FilterThreadCompanion()
	{
		{
			std::unique_lock<std::mutex> lLock(mMutex);
			mQuit = true;
			mWaitCondition.notify_all();
			mWaitConditionResult.notify_all();
		}
		waitThread();
	}
	
	void setChannelToFilter(const cv::Mat & pMat)
	{
		std::unique_lock<std::mutex> lLock(mMutex);
		mInput = &pMat;
		mWaitCondition.notify_all();
	}
	
	void getResult(cv::Mat & pRes)
	{
		std::unique_lock<std::mutex> lLock(mMutex);
		if(mQuit)
			return;
		if(mInput)
		{
			// job was not take by the worker !!!
			mWaitConditionResult.wait(lLock);
		}
		cv::swap(pRes,mResult);
	}
	
protected:
	virtual void run()
	{
		cv::Mat lTmp;
		std::unique_lock<std::mutex> lLock(mMutex);
		while(!mQuit)
		{
			if(!mInput)
			{
				mWaitCondition.wait(lLock);
				continue;
			}
			
			processChannel(*mInput,mMorphoKernel,mResult,lTmp,mParameters,true);

			mInput = NULL;
			mWaitConditionResult.notify_all();
		}
	}
private:
	const FilterThread::ChannelParameters &		mParameters;
	const cv::Mat & 			mMorphoKernel;
	std::mutex					mMutex;
	std::condition_variable		mWaitCondition;
	std::condition_variable		mWaitConditionResult;
	cv::Mat const *				mInput;
	cv::Mat						mResult;
	bool						mQuit;
};

void FilterThread::run()
{
	GrabbedFrame lFrame;
	
	const cv::Mat lMorphoKernel5x5 = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(5,5));
	
	
	cv::Mat lTmpA(mCameraThread.mParameters.mHalfHeight,mCameraThread.mParameters.mHalfWidth,CV_8UC1);
	cv::Mat lTmpB(mCameraThread.mParameters.mHalfHeight,mCameraThread.mParameters.mHalfWidth,CV_8UC1);
	cv::Mat lTmpC(mCameraThread.mParameters.mHalfHeight,mCameraThread.mParameters.mHalfWidth,CV_8UC1);
	
	FilterThreadCompanion lUCompanion(mParameters.mUParam,lMorphoKernel5x5);
	FilterThreadCompanion lVCompanion(mParameters.mVParam,lMorphoKernel5x5);
	
	while(!mQuit)
	{
		if(!mCameraThread.getNextFrame(lFrame))
			continue;
		lFrame.setTimestamp(GrabbedFrame::F_FilterStart);
		lVCompanion.setChannelToFilter(lFrame[GrabbedFrame::V]);
		lUCompanion.setChannelToFilter(lFrame[GrabbedFrame::U]);
		processChannel(lFrame[GrabbedFrame::Y], lMorphoKernel5x5,lTmpC,lFrame[GrabbedFrame::BackgroundMask],mParameters.mYParam,false);
		lVCompanion.getResult(lTmpA);
		lUCompanion.getResult(lTmpB);
		cv::bitwise_and(lTmpB,lTmpA,lFrame[GrabbedFrame::BackgroundMask]);
		cv::bitwise_not(lFrame[GrabbedFrame::BackgroundMask],lTmpA);
		cv::bitwise_or(lTmpA,lTmpC,lFrame[GrabbedFrame::BackgroundMask]);
		if(mParameters.mErode)
		{
			cv::morphologyEx(lFrame[GrabbedFrame::BackgroundMask],lTmpA,cv::MORPH_ERODE,lMorphoKernel5x5);
			cv::swap(lFrame[GrabbedFrame::BackgroundMask],lTmpA);
		}
		
		// clear top and bottom (we capture border of camera support)
		if(mParameters.mExclusionZoneTopPercent)
		{
			const cv::Point lTopEdgeRectA = cv::Point(0,0);
			const cv::Point lTopEdgeRectB = cv::Point(mCameraThread.mParameters.mHalfWidth,(int)(mCameraThread.mParameters.mHalfHeight * mParameters.mExclusionZoneTopPercent));
			cv::rectangle(lFrame[GrabbedFrame::BackgroundMask],lTopEdgeRectA,lTopEdgeRectB,0,CV_FILLED);
		}
		if(mParameters.mExclusionZoneBottomPercent)
		{
			const cv::Point lBottomEdgeRectA = cv::Point(0,mCameraThread.mParameters.mHalfHeight - 1 - (int)(mCameraThread.mParameters.mHalfHeight * mParameters.mExclusionZoneBottomPercent));
			const cv::Point lBottomEdgeRectB = cv::Point(mCameraThread.mParameters.mHalfWidth,mCameraThread.mParameters.mHalfHeight - 1);
			cv::rectangle(lFrame[GrabbedFrame::BackgroundMask],lBottomEdgeRectA,lBottomEdgeRectB,0,CV_FILLED);
		}
		
		lFrame.setTimestamp(GrabbedFrame::F_FilterDone);
		
		{
			std::unique_lock<std::mutex> lLock(mMutex);
			if(mQuit)
				continue;
			mFrame.swap(lFrame);
			mGotNewFrame = true;
			mWaitCondition.notify_all();
			// and wait until frame is consumed
			mWaitCondition.wait(lLock);
		}
		
	}
}
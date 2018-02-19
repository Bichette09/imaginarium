#include "filter_thread.h"

FilterThread::Parameters::Parameters()
	: mThresholdU(-1)
	, mThresholdV(-1)
	, mGaussian(false)
	, mDilate(false)
	, mErode(false)
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

void processChannel(const cv::Mat & pIn, const cv::Mat & pMorphoKernel, cv::Mat & pOut, cv::Mat & pTmp, float pTh, bool pBlur, bool pDilate)
{
	cv::Mat const * lThIn = &pIn;
	if(pBlur)
	{
		cv::GaussianBlur(pIn,pTmp,cv::Size(5,5),0);
		lThIn = &pTmp;
	}
	cv::threshold(*lThIn,pOut,pTh,255,cv::THRESH_BINARY_INV + (pTh < 0 ? cv::THRESH_OTSU : 0));
	if(pDilate)
	{
		// because here mask is inverted we should do erode
		cv::morphologyEx(pOut,pTmp,cv::MORPH_ERODE,pMorphoKernel);
		cv::swap(pTmp,pOut);
	}
}

void FilterThread::run()
{
	GrabbedFrame lFrame;
	
	const cv::Mat lMorphoKernel = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(5,5));
	const cv::Point lTopEdgeRectA = cv::Point(0,0);
	const cv::Point lTopEdgeRectB = cv::Point(mCameraThread.mParameters.mHalfWidth,(int)(mCameraThread.mParameters.mHalfHeight * 0.06));
	const cv::Point lBottomEdgeRectA = cv::Point(0,mCameraThread.mParameters.mHalfHeight - 1 - (int)(mCameraThread.mParameters.mHalfHeight * 0.025));
	const cv::Point lBottomEdgeRectB = cv::Point(mCameraThread.mParameters.mHalfWidth,mCameraThread.mParameters.mHalfHeight - 1);
	
	cv::Mat lTmpA(mCameraThread.mParameters.mHalfHeight,mCameraThread.mParameters.mHalfWidth,CV_8UC1);
	cv::Mat lTmpB(mCameraThread.mParameters.mHalfHeight,mCameraThread.mParameters.mHalfWidth,CV_8UC1);
	
	while(!mQuit)
	{
		if(!mCameraThread.getNextFrame(lFrame))
			continue;
		
		processChannel(lFrame[GrabbedFrame::U],lMorphoKernel,lTmpA,lFrame[GrabbedFrame::BackgroundMask],mParameters.mThresholdU,mParameters.mGaussian,mParameters.mDilate);
		processChannel(lFrame[GrabbedFrame::V],lMorphoKernel,lTmpB,lFrame[GrabbedFrame::BackgroundMask],mParameters.mThresholdV,mParameters.mGaussian,mParameters.mDilate);
		
		cv::bitwise_and(lTmpB,lTmpA,lFrame[GrabbedFrame::BackgroundMask]);
		cv::bitwise_not(lFrame[GrabbedFrame::BackgroundMask],lTmpA);
		if(mParameters.mErode)
		{
			cv::morphologyEx(lTmpA,lFrame[GrabbedFrame::BackgroundMask],cv::MORPH_ERODE,lMorphoKernel);
		}
		else
		{
			cv::swap(lFrame[GrabbedFrame::BackgroundMask],lTmpA);
		}
		
		// clear top and bottom (we capture border of camera support)
		cv::rectangle(lFrame[GrabbedFrame::BackgroundMask],lTopEdgeRectA,lTopEdgeRectB,0,CV_FILLED);
		cv::rectangle(lFrame[GrabbedFrame::BackgroundMask],lBottomEdgeRectA,lBottomEdgeRectB,0,CV_FILLED);
		
		
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
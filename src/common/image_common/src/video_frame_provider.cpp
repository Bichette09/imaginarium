#include "video_frame_provider.h"

// std
#include <algorithm>

// ros
#include "ros/ros.h"

//std::max(1,std::min(pFps,50))
//std::max(320,std::min(pOutputWidth,3200))
// std::max(240,std::min(pOutputHeight,2400))

VideoFrameProvider::Parameters::Parameters(int pOutputWidth, int pOutputHeight, int pFps, const std::string & pSrcFile)
	: mSrcFile(pSrcFile)
	, mWidth( std::max(320,std::min(pOutputWidth,3200)))
	, mHeight( std::max(240,std::min(pOutputHeight,2400)))
	, mFps(std::max(1,std::min(pFps,50)))
	, mFramePeriodMs(1000/mFps)
{
	
}

VideoFrameProvider::Parameters::~Parameters()
{
}

VideoFrameProvider::VideoFrameProvider(Parameters pParams)
	: mParameters(pParams)
	, mIsError(false)
	, mCaptureHandle(NULL)
{
	mTmpCvtArray.resize(3);
}

VideoFrameProvider::~VideoFrameProvider()
{
	if(mCaptureHandle)
		mCaptureHandle->release();
	delete mCaptureHandle; mCaptureHandle = NULL;
}

bool VideoFrameProvider::getNextFrame(FrameInterface & pRes)
{
	if(mIsError)
		return false;
	
	if(!mCaptureHandle)
	{
		// open stream
		mCaptureHandle = new cv::VideoCapture(mParameters.mSrcFile);
		if(!mCaptureHandle || !mCaptureHandle->isOpened())
		{
			mIsError = true;
			ROS_WARN_STREAM("fail to open video "<<mParameters.mSrcFile);
			return false;
		}
	}
	
	*mCaptureHandle>>mTmpA;
	if(mTmpA.empty())
	{
		mCaptureHandle->release();
		delete mCaptureHandle;
		mCaptureHandle = NULL;
		return false;
	}
	
	std::chrono::time_point<std::chrono::system_clock> lNow = std::chrono::system_clock::now();
	int lElapsedMs = std::chrono::duration<float,std::milli>(lNow - mLastGrab).count();
	mLastGrab = lNow;
	if(lElapsedMs < mParameters.mFramePeriodMs)
	{
		usleep( (mParameters.mFramePeriodMs - lElapsedMs)*1000);
	}
	
	cv::resize(mTmpA,mTmpB,cv::Size(mParameters.mWidth,mParameters.mHeight));
	cvtColor(mTmpB, mTmpA, cv::COLOR_BGR2YUV);
	
	// mYuvFrame = mRgbFrame;
	cv::split(mTmpA,mTmpCvtArray);
	pRes.editY() = mTmpCvtArray[0];
	pRes.editU() = mTmpCvtArray[1];
	pRes.editV() = mTmpCvtArray[2];
	
	pRes.setGrabTimestamp();
	
	return true;
}

int VideoFrameProvider::getFrameWidth() const
{
	return mParameters.mWidth;
}

int VideoFrameProvider::getFrameHeight() const
{
	return mParameters.mHeight;
}
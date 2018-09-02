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
	, mSleepUs(1000000/mFps)
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
	
	usleep(mParameters.mSleepUs);
	
	*mCaptureHandle>>mTmpA;
	if(mTmpA.empty())
	{
		mCaptureHandle->release();
		delete mCaptureHandle;
		mCaptureHandle = NULL;
		return false;
	}
	
	cv::resize(mTmpA,mTmpB,cv::Size(mParameters.mWidth,mParameters.mHeight));
	cvtColor(mTmpB, mTmpA, cv::COLOR_BGR2YUV);
	
	// mYuvFrame = mRgbFrame;
	std::vector<cv::Mat> lOutputArray(3);
	cv::split(mTmpA,lOutputArray);
	pRes.editY() = lOutputArray[0];
	pRes.editU() = lOutputArray[1];
	pRes.editV() = lOutputArray[2];
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
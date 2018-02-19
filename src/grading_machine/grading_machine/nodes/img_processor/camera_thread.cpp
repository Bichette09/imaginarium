#include "camera_thread.h"

// opencv
#include "opencv2/opencv.hpp"

// raspicam lib
#include <raspicam/raspicam.h>

// ros
#include "ros/ros.h"

CameraThread::Parameters::Parameters(int pRequestedWidth, int pRequestedHeight, int pFps)
	: mWidth( std::max(320,std::min(320 * (int)std::floor(pRequestedWidth/320.),3200)))
	, mHeight( std::max(240,std::min(240 * (int)std::floor(pRequestedWidth/240.),2400)))
	, mHalfWidth(mWidth / 2)
	, mHalfHeight(mHeight / 2)
	, mPixelCount(mWidth*mHeight)
	, mQuarterPixelCount(mHalfWidth*mHalfHeight)
	, mFps(std::max(1,std::min(pFps,50)))
{
}

CameraThread::Parameters::~Parameters()
{
}

CameraThread::CameraThread(Parameters pParams)
	: mParameters(pParams)
	, mQuit(false)
	, mIsCapturing(false)
	, mIsError(false)
	, mGotNewFrame(false)
{
	EnsureMatSizeAndType(mFrame,mParameters);
	startThread();
}

CameraThread::~CameraThread()
{
	{
		std::unique_lock<std::mutex> lLock(mMutex);
		mQuit = true;
		mWaitCondition.notify_all();
	}
	waitThread();
}

bool CameraThread::getNextFrame(GrabbedFrame & pFrame,int pTimeoutMs)
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

bool CameraThread::isCapturing() const
{
	return mIsCapturing;
}

bool CameraThread::isOk() const
{
	return !mIsError;
}

bool CameraThread::waitForCapture() const
{
	while(!mIsCapturing && !mIsError)
	{
		usleep(15000);
	}
	return mIsCapturing && !mIsError;
}


void CameraThread::EnsureMatSizeAndType(GrabbedFrame & pFrame, const Parameters & pParams)
{
	// ensure that mat have the right format
	if(pFrame[GrabbedFrame::Y].type() != CV_8UC1 || pFrame[GrabbedFrame::Y].size[1] != pParams.mWidth || pFrame[GrabbedFrame::Y].size[0] != pParams.mHeight)
	{
		pFrame[GrabbedFrame::Y] = cv::Mat(pParams.mHeight,pParams.mWidth,CV_8UC1);
	}
	if(pFrame[GrabbedFrame::U].type() != CV_8UC1 || pFrame[GrabbedFrame::U].size[1] != pParams.mHalfWidth || pFrame[GrabbedFrame::U].size[0] != pParams.mHalfHeight)
	{
		pFrame[GrabbedFrame::U] = cv::Mat(pParams.mHalfHeight,pParams.mHalfWidth,CV_8UC1);
	}
	if(pFrame[GrabbedFrame::V].type() != CV_8UC1 || pFrame[GrabbedFrame::V].size[1] != pParams.mHalfWidth || pFrame[GrabbedFrame::V].size[0] != pParams.mHalfHeight)
	{
		pFrame[GrabbedFrame::V] = cv::Mat(pParams.mHalfHeight,pParams.mHalfWidth,CV_8UC1);
	}
}

void CameraThread::run()
{
	GrabbedFrame lFrame;
	
	raspicam::RaspiCam lCameraHandle;
	// we capture in YUV to achieve max speed
	lCameraHandle.setFormat( raspicam::RASPICAM_FORMAT_YUV420);
	// lCameraHandle.setAWB(raspicam::RASPICAM_AWB_FLASH);
	//lCameraHandle.setExposure(raspicam::RASPICAM_EXPOSURE_SPOTLIGHT);
	lCameraHandle.setCaptureSize(mParameters.mWidth,mParameters.mHeight);
	lCameraHandle.setFrameRate( mParameters.mFps );
	
	unsigned char * lBuffer = NULL;
	//Open camera 
	if ( !lCameraHandle.open()) 
	{
		ROS_ERROR_STREAM("CameraThread fail to open video stream");
		mIsError = true;
	}
	else
	{
		sleep(3.);
		mIsCapturing = true;
		size_t lBufferSize = lCameraHandle.getImageTypeSize ( lCameraHandle.getFormat() );
		
		lBuffer = new unsigned char[ lBufferSize];
		ROS_INFO_STREAM("CameraThread capture Y["<<mParameters.mWidth<<"x"<<mParameters.mHeight<<"] UV["<<mParameters.mHalfWidth<<"x"<<mParameters.mHalfHeight<<"] @"<<mParameters.mFps<<"fps");
	}
	
	while(!mQuit && !mIsError)
	{

		EnsureMatSizeAndType(lFrame,mParameters);

		lCameraHandle.grab();
		lCameraHandle.retrieve ( lBuffer);
		lFrame.setTimestamp();
		memcpy(lFrame[GrabbedFrame::Y].data,lBuffer,mParameters.mPixelCount);
		memcpy(lFrame[GrabbedFrame::U].data,lBuffer + mParameters.mPixelCount, mParameters.mQuarterPixelCount);
		memcpy(lFrame[GrabbedFrame::V].data,lBuffer + mParameters.mPixelCount +  mParameters.mQuarterPixelCount, mParameters.mQuarterPixelCount);
	
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
	delete lBuffer;
	lBuffer = NULL;
}
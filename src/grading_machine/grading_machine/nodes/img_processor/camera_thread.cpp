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
	
	mInterThreadData[0] = new cv::Mat();
	mInterThreadData[1] = new cv::Mat();
	mInterThreadData[2] = new cv::Mat();
	EnsureMatSizeAndType(*mInterThreadData[0],*mInterThreadData[1],*mInterThreadData[2],mParameters);
	startThread();
}

CameraThread::~CameraThread()
{
	mMutex.lock();
	mQuit = true;
	mMutex.unlock();
	waitThread();
	
	for(int i = 0 ; i < 3 ; ++i)
	{
		delete mInterThreadData[i];
		mInterThreadData[i] = NULL;
	}
}

void CameraThread::getFrame(cv::Mat & pY, cv::Mat & pU, cv::Mat & pV)
{
	mMutex.lock();
	cv::swap(pY,*mInterThreadData[0]);
	cv::swap(pU,*mInterThreadData[1]);
	cv::swap(pV,*mInterThreadData[2]);
	mGotNewFrame = false;
	mMutex.unlock();
}

bool CameraThread::isCapturing() const
{
	return mIsCapturing;
}

bool CameraThread::isOk() const
{
	return !mIsError;
}

bool CameraThread::hasNewFrame() const
{
	return mGotNewFrame;
}

bool CameraThread::waitForCapture() const
{
	while(!mIsCapturing && !mIsError)
	{
		usleep(15000);
	}
	return mIsCapturing && !mIsError;
}

void CameraThread::EnsureMatSizeAndType(cv::Mat & pY, cv::Mat & pU, cv::Mat & pV, const Parameters & pParams)
{
	// ensure that mat have the right format
	if(pY.type() != CV_8UC1 || pY.size[1] != pParams.mWidth || pY.size[0] != pParams.mHeight)
	{
		pY = cv::Mat(pParams.mHeight,pParams.mWidth,CV_8UC1);
	}
	if(pU.type() != CV_8UC1 || pU.size[1] != pParams.mHalfWidth || pU.size[0] != pParams.mHalfHeight)
	{
		pU = cv::Mat(pParams.mHalfHeight,pParams.mHalfWidth,CV_8UC1);
	}
	if(pV.type() != CV_8UC1 || pV.size[1] != pParams.mHalfWidth || pV.size[0] != pParams.mHalfHeight)
	{
		pV = cv::Mat(pParams.mHalfHeight,pParams.mHalfWidth,CV_8UC1);
	}
}

void CameraThread::run()
{
	cv::Mat lY;
	cv::Mat lU;
	cv::Mat lV;
	
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
		EnsureMatSizeAndType(lY,lU,lV,mParameters);

		lCameraHandle.grab();
		lCameraHandle.retrieve ( lBuffer);

		memcpy(lY.data,lBuffer,mParameters.mPixelCount);
		memcpy(lU.data,lBuffer + mParameters.mPixelCount, mParameters.mQuarterPixelCount);
		memcpy(lV.data,lBuffer + mParameters.mPixelCount +  mParameters.mQuarterPixelCount, mParameters.mQuarterPixelCount);
	
		mMutex.lock();
		cv::swap(lY,*mInterThreadData[0]);
		cv::swap(lU,*mInterThreadData[1]);
		cv::swap(lV,*mInterThreadData[2]);
		mGotNewFrame = true;
		mMutex.unlock();
	
	}
	
	delete lBuffer;
	lBuffer = NULL;
}
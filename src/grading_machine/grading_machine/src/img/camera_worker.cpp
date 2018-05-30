#include "camera_worker.h"

// raspicam lib
#include <raspicam/raspicam.h>

// ros
#include "ros/ros.h"

CameraWorker::Parameters::Parameters(int pRequestedWidth, int pRequestedHeight, int pFps)
	: mWidth( std::max(320,std::min(320 * (int)std::floor(pRequestedWidth/320.),3200)))
	, mHeight( std::max(240,std::min(240 * (int)std::floor(pRequestedWidth/240.),2400)))
	, mHalfWidth(mWidth / 2)
	, mHalfHeight(mHeight / 2)
	, mPixelCount(mWidth*mHeight)
	, mQuarterPixelCount(mHalfWidth*mHalfHeight)
	, mFps(std::max(1,std::min(pFps,50)))
	
{
}

CameraWorker::Parameters::~Parameters()
{
}

CameraWorker::CameraWorker(Parameters pParams)
	: mParameters(pParams)
	, mIsError(false)
	, mBuffer(NULL)
	
{
	mCameraHandle = new raspicam::RaspiCam();
	// we capture in YUV to achieve max speed
	mCameraHandle->setFormat( raspicam::RASPICAM_FORMAT_YUV420);
	// mCameraHandle->setAWB(raspicam::RASPICAM_AWB_FLASH);
	// mCameraHandle->setExposure(raspicam::RASPICAM_EXPOSURE_SPOTLIGHT);
	mCameraHandle->setCaptureSize(mParameters.mWidth,mParameters.mHeight);
	mCameraHandle->setFrameRate( mParameters.mFps );
	
	unsigned char * lBuffer = NULL;
	//Open camera 
	if ( !mCameraHandle->open()) 
	{
		ROS_ERROR_STREAM("CameraThread fail to open video stream");
		mIsError = true;
	}
	else
	{
		sleep(3.);
		size_t lBufferSize = mCameraHandle->getImageTypeSize ( mCameraHandle->getFormat() );
		
		mBuffer = new unsigned char[ lBufferSize];
		ROS_INFO_STREAM("OpenCV version "<<CV_MAJOR_VERSION<<"."<<CV_MINOR_VERSION);
		ROS_INFO_STREAM("CameraThread capture Y["<<mParameters.mWidth<<"x"<<mParameters.mHeight<<"] UV["<<mParameters.mHalfWidth<<"x"<<mParameters.mHalfHeight<<"] @"<<mParameters.mFps<<"fps");
	}
	
	mFullY = cv::Mat(mParameters.mHeight,mParameters.mWidth,CV_8UC1);
}

CameraWorker::~CameraWorker()
{
	delete mBuffer;
	mBuffer = NULL;
}

void CameraWorker::EnsureMatSizeAndType(Frame & pFrame, const Parameters & pParams)
{
	// ensure that mat have the right format
	if(pFrame[Frame::Y].type() != CV_8UC1 || pFrame[Frame::Y].size[1] != pParams.mHalfWidth || pFrame[Frame::Y].size[0] != pParams.mHalfHeight)
	{
		pFrame[Frame::Y] = cv::Mat(pParams.mHalfHeight,pParams.mHalfWidth,CV_8UC1);
	}
	if(pFrame[Frame::U].type() != CV_8UC1 || pFrame[Frame::U].size[1] != pParams.mHalfWidth || pFrame[Frame::U].size[0] != pParams.mHalfHeight)
	{
		pFrame[Frame::U] = cv::Mat(pParams.mHalfHeight,pParams.mHalfWidth,CV_8UC1);
	}
	if(pFrame[Frame::V].type() != CV_8UC1 || pFrame[Frame::V].size[1] != pParams.mHalfWidth || pFrame[Frame::V].size[0] != pParams.mHalfHeight)
	{
		pFrame[Frame::V] = cv::Mat(pParams.mHalfHeight,pParams.mHalfWidth,CV_8UC1);
	}
}

bool CameraWorker::computeNextResult(Frame & pRes)
{
	if(mIsError)
		return false;
	EnsureMatSizeAndType(pRes,mParameters);

	
	mCameraHandle->grab();
	mCameraHandle->retrieve ( mBuffer );
	
	pRes.setTimestamp(Frame::F_GrabDone);
	memcpy(mFullY.data,mBuffer,mParameters.mPixelCount);
	cv::resize(mFullY,pRes[Frame::Y],cv::Size(mParameters.mHalfWidth,mParameters.mHalfHeight));
	memcpy(pRes[Frame::U].data,mBuffer + mParameters.mPixelCount, mParameters.mQuarterPixelCount);
	memcpy(pRes[Frame::V].data,mBuffer + mParameters.mPixelCount +  mParameters.mQuarterPixelCount, mParameters.mQuarterPixelCount);
	
	return true;
}


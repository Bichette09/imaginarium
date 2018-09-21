#include "camera_frame_provider.h"

// raspicam lib
#ifdef USE_RASPICAM_LIB
#include <raspicam/raspicam.h>
#endif

// ros
#include "ros/ros.h"

CameraFrameProvider::Parameters::Parameters(int pRequestedWidth, int pRequestedHeight, int pFps)
	: mCaptureWidth( std::max(320,std::min(320 * (int)std::floor(pRequestedWidth*2/320.),3200)))
	, mCaptureHeight( std::max(240,std::min(240 * (int)std::floor(pRequestedHeight*2/240.),2400)))
	, mHalfWidth(mCaptureWidth / 2)
	, mHalfHeight(mCaptureHeight / 2)
	, mPixelCount(mCaptureWidth*mCaptureHeight)
	, mQuarterPixelCount(mHalfWidth*mHalfHeight)
	, mFps(std::max(1,std::min(pFps,50)))
	
{
}

CameraFrameProvider::Parameters::~Parameters()
{
}

CameraFrameProvider::CameraFrameProvider(Parameters pParams)
	: mParameters(pParams)
	, mIsError(false)
	, mBuffer(NULL)
	, mCameraHandle(NULL)
{
#ifdef USE_RASPICAM_LIB
	mCameraHandle = new raspicam::RaspiCam();
	// we capture in YUV to achieve max speed
	mCameraHandle->setFormat( raspicam::RASPICAM_FORMAT_YUV420);
	// mCameraHandle->setAWB(raspicam::RASPICAM_AWB_FLASH);
	// mCameraHandle->setExposure(raspicam::RASPICAM_EXPOSURE_SPOTLIGHT);
	mCameraHandle->setCaptureSize(mParameters.mCaptureWidth,mParameters.mCaptureHeight);
	mCameraHandle->setFrameRate( mParameters.mFps );
	
	unsigned char * lBuffer = NULL;
	//Open camera 
	if ( !mCameraHandle->open()) 
	{
		ROS_ERROR_STREAM("CameraThread fail to open video stream (ensure that picam interface is enabled and video core as enough memory)");
		mIsError = true;
	}
	else
	{
		sleep(3.);
		size_t lBufferSize = mCameraHandle->getImageTypeSize ( mCameraHandle->getFormat() );
		
		mBuffer = new unsigned char[ lBufferSize];
		ROS_INFO_STREAM("OpenCV version "<<CV_MAJOR_VERSION<<"."<<CV_MINOR_VERSION);
		ROS_INFO_STREAM("CameraThread capture Y["<<mParameters.mCaptureWidth<<"x"<<mParameters.mCaptureHeight<<"] UV["<<mParameters.mHalfWidth<<"x"<<mParameters.mHalfHeight<<"] @"<<mParameters.mFps<<"fps, output resolution ["<<mParameters.mHalfWidth<<"x"<<mParameters.mHalfHeight<<"]");
	}
#else
	ROS_ERROR_STREAM("CameraThread not build with libraspicam support");
	mIsError = true;
#endif
	
	mFullY = cv::Mat(mParameters.mCaptureHeight,mParameters.mCaptureWidth,CV_8UC1);
}

CameraFrameProvider::~CameraFrameProvider()
{
#ifdef USE_RASPICAM_LIB
	delete mCameraHandle;
	mCameraHandle = NULL;
#endif
	delete mBuffer;
	mBuffer = NULL;
}

void CameraFrameProvider::EnsureMatSizeAndType(FrameInterface & pFrame, const Parameters & pParams)
{
	pFrame.ensureSizeAndType(pParams.mHalfWidth,pParams.mHalfHeight);
}

bool CameraFrameProvider::getNextFrame(FrameInterface & pRes)
{
#ifdef USE_RASPICAM_LIB
	if(mIsError)
		return false;
	EnsureMatSizeAndType(pRes,mParameters);


	mCameraHandle->grab();
	mCameraHandle->retrieve ( mBuffer );

	pRes.setGrabTimestamp();
	memcpy(mFullY.data,mBuffer,mParameters.mPixelCount);
	cv::resize(mFullY,pRes.editY(),cv::Size(mParameters.mHalfWidth,mParameters.mHalfHeight));
	memcpy(pRes.editU().data,mBuffer + mParameters.mPixelCount, mParameters.mQuarterPixelCount);
	memcpy(pRes.editV().data,mBuffer + mParameters.mPixelCount +  mParameters.mQuarterPixelCount, mParameters.mQuarterPixelCount);
	
	return true;
#else
	return false;
#endif	
}

int CameraFrameProvider::getFrameWidth() const
{
	return mParameters.mHalfWidth;
}

int CameraFrameProvider::getFrameHeight() const
{
	return mParameters.mHalfHeight;
}
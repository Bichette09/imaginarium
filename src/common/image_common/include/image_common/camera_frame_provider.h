#pragma once


#include "image_common/frame_interface.h"

// std

namespace raspicam
{
	class RaspiCam;
}

class CameraFrameProvider : public FrameProvider
{
public:
	
	
	
	struct Parameters
	{
	public:
		/** @param pRequestedWidth should be a multiple of 320
		*	@param pRequestedWidth should be a multiple of 240
		*/
		Parameters(int pRequestedWidth, int pRequestedHeight, int pFps);
		~Parameters();
		
		/** capture resolution of the camera (full for Y, half for UV)
		*/
		const int	mCaptureWidth;
		const int	mCaptureHeight;
		/** resolution that will be available for output, Y will be downscale to match UV resolution
		*/
		const int	mHalfWidth;
		const int	mHalfHeight;
		const int	mPixelCount;
		const int	mQuarterPixelCount;
		const int 	mFps;
		
	private:
		
	};
	
	CameraFrameProvider(Parameters pParams);
	virtual ~CameraFrameProvider();
	
	
	static void EnsureMatSizeAndType(FrameInterface & pFrame, const Parameters & pParams);
	
	const Parameters	mParameters;

	virtual bool getNextFrame(FrameInterface & pRes);
	virtual int getFrameWidth() const;
	virtual int getFrameHeight() const;
	
private:
	raspicam::RaspiCam *	mCameraHandle;
	unsigned char *			mBuffer;
	cv::Mat 				mFullY;
	bool					mIsError;
};
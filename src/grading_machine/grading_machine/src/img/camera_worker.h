#pragma once


#include "frame_processor.h"

// std

namespace raspicam
{
	class RaspiCam;
}

class CameraWorker : public FrameProcessor::WorkerInterface
{
public:
	
	struct Parameters
	{
	public:
		Parameters(int pRequestedWidth, int pRequestedHeight, int pFps);
		~Parameters();
		
		const int	mWidth;
		const int	mHeight;
		const int	mHalfWidth;
		const int	mHalfHeight;
		const int	mPixelCount;
		const int	mQuarterPixelCount;
		const int 	mFps;
		
	private:
		
	};
	
	/** pWidth should be a multiple of 320
	*	pHeight should be a multiple of 240
	*/
	CameraWorker(Parameters pParams);
	virtual ~CameraWorker();
	
	
	static void EnsureMatSizeAndType(Frame & pFrame, const Parameters & pParams);
	
	const Parameters	mParameters;

protected:
	virtual bool computeNextResult(Frame & pRes);
private:
	raspicam::RaspiCam *	mCameraHandle;
	unsigned char *			mBuffer;
	cv::Mat 				mFullY;
	bool					mIsError;
};
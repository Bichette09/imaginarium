#pragma once


#include "image_common/frame_interface.h"

// opencv
#include "opencv2/opencv.hpp"

// std

class VideoFrameProvider : public FrameProvider
{
public:
	
	
	
	struct Parameters
	{
	public:
		Parameters(int pOutputWidth, int pOutputHeight, int pFps, const std::string & pSrcFile);
		~Parameters();
		
		const std::string mSrcFile;
		const int	mWidth;
		const int	mHeight;
		const int 	mFps;
		const int	mSleepUs;
		
	private:
		
	};
	
	VideoFrameProvider(Parameters pParams);
	virtual ~VideoFrameProvider();
	
	const Parameters	mParameters;

	virtual bool getNextFrame(FrameInterface & pRes);
	virtual int getFrameWidth() const;
	virtual int getFrameHeight() const;
	
private:
	cv::VideoCapture * mCaptureHandle;
	cv::Mat mTmpA;
	cv::Mat mTmpB;
	bool	mIsError;
};
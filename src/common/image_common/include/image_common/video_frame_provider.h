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
		const int	mFramePeriodMs;
		
	private:
		
	};
	
	VideoFrameProvider(Parameters pParams);
	virtual ~VideoFrameProvider();
	
	const Parameters	mParameters;

	virtual bool getNextFrame(FrameInterface & pRes);
	virtual int getFrameWidth() const;
	virtual int getFrameHeight() const;
	
private:
	std::vector<cv::Mat> 	mTmpCvtArray;
	std::chrono::time_point<std::chrono::system_clock>	mLastGrab;
	cv::VideoCapture * mCaptureHandle;
	cv::Mat mTmpA;
	cv::Mat mTmpB;
	bool	mIsError;
};
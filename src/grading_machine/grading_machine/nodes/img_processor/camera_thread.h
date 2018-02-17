#pragma once


#include "thread_interface.h"

// std
#include <mutex>

namespace cv
{
	class Mat;
}

class CameraThread : public ThreadInterface
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
	CameraThread(Parameters pParams);
	virtual ~CameraThread();
	
	bool waitForCapture() const;
	
	/** return last capture data 
	*	pY is full res, pU and pV will be a quarter of the resolution
	*/
	void getFrame(cv::Mat & pY, cv::Mat & pU, cv::Mat & pV);
	
	bool isCapturing() const;
	bool isOk() const;
	bool hasNewFrame() const;
	
	static void EnsureMatSizeAndType(cv::Mat & pY, cv::Mat & pU, cv::Mat & pV, const Parameters & pParams);
	
	const Parameters	mParameters;
	
	
	
protected:
	virtual void run();
private:
	std::mutex	mMutex;
	
	cv::Mat*	mInterThreadData[3];
	bool		mGotNewFrame;
	
	bool		mIsCapturing;
	bool		mIsError;
	bool		mQuit;
};
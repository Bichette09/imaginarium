#pragma once


#include "thread_interface.h"

// std
#include <mutex>
#include <condition_variable>

#include "grabbed_frame.h"

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
	bool getNextFrame(GrabbedFrame & pFrame,int pTimeoutMs = 50);
	
	bool isCapturing() const;
	bool isOk() const;
	
	static void EnsureMatSizeAndType(GrabbedFrame & pFrame, const Parameters & pParams);
	
	const Parameters	mParameters;

protected:
	virtual void run();
private:
	std::mutex					mMutex;
	std::condition_variable		mWaitCondition;
	GrabbedFrame	mFrame;
	bool		mGotNewFrame;
	
	bool		mIsCapturing;
	bool		mIsError;
	bool		mQuit;
};
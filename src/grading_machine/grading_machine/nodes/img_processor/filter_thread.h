#pragma once


#include "camera_thread.h"

/** this thread will build mask layer
*/
class FilterThread : public ThreadInterface
{
public:
	
	struct Parameters
	{
		Parameters();
		
		int 	mThresholdU;
		int 	mThresholdV;
		bool	mGaussian;
		bool	mDilate;
		bool	mErode;
		
	};
	
	FilterThread(CameraThread & pCameraThread, const Parameters & pParameters);
	virtual ~FilterThread();
	
	bool getNextFrame(GrabbedFrame & pFrame,int pTimeoutMs = 50);
	
protected:
	virtual void run();
private:
	std::mutex					mMutex;
	std::condition_variable		mWaitCondition;
	CameraThread & 				mCameraThread;
	GrabbedFrame				mFrame;
	const Parameters &			mParameters;
	bool						mGotNewFrame;
	
	bool						mQuit;
};
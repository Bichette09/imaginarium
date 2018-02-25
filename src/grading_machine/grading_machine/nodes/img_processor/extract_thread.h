#pragma once


#include "filter_thread.h"

/** this thread will extract area of interest 
*/
class ExtractThread : public ThreadInterface
{
public:
	
	struct Parameters
	{
		Parameters();
		
		int		mMinimumPixelsPerGroup;
		float	mMinimumBackgroundPercent;
		float	mMinimumSpaceBetweenAreaPercent;
		bool	mConnectivityFullWay;
	};
	
	ExtractThread(FilterThread & pFilterThread, const Parameters & pParameters);
	virtual ~ExtractThread();
	
	bool getNextFrame(GrabbedFrame & pFrame,int pTimeoutMs = 50);
	
protected:
	virtual void run();
private:
	std::mutex					mMutex;
	std::condition_variable		mWaitCondition;
	FilterThread & 				mFilterThread;
	GrabbedFrame				mFrame;
	const Parameters &			mParameters;
	bool						mGotNewFrame;
	
	bool						mQuit;
};
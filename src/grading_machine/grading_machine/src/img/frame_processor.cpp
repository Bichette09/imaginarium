#include "frame_processor.h"

// ros
#include "ros/ros.h"

#define USE_THREAD

FrameProcessor::WorkerInterface::WorkerInterface()
{
}

FrameProcessor::WorkerInterface::~WorkerInterface()
{
}

FrameProcessor::FrameProcessor(WorkerInterface & pWorker)
	: mQuit(false)
	, mGotNewResult(false)
	, mWorker(pWorker)
{
#ifdef USE_THREAD
	startThread();
#endif
}

FrameProcessor::~FrameProcessor()
{
#ifdef USE_THREAD
	{
		std::unique_lock<std::mutex> lLock(mMutex);
		mQuit = true;
		mWaitCondition.notify_all();
	}
	waitThread();
#endif
}

bool FrameProcessor::getNextFrame(Frame & pFrame)
{
#ifdef USE_THREAD
	std::unique_lock<std::mutex> lLock(mMutex);
	if(!mGotNewResult)
	{
		mWaitCondition.wait_until(lLock,std::chrono::system_clock::now() + std::chrono::milliseconds(500));
	}
	if(!mGotNewResult)
	{
		ROS_WARN_STREAM("could not get next frame !!");
		return false;
	}
	mNextResult.swap(pFrame);
	mGotNewResult = false;
	mWaitCondition.notify_all();
	return true;
#else
	if(!mWorker.computeNextResult(mTmpFrame))
		return false;
	mTmpFrame.swap(pFrame);
	return true;
#endif
}


void FrameProcessor::run()
{
	while(!mQuit)
	{
		if(mWorker.computeNextResult(mTmpFrame))
		{
			std::unique_lock<std::mutex> lLock(mMutex);
			if(mQuit)
				continue;
			mTmpFrame.swap(mNextResult);
			mGotNewResult = true;
			mWaitCondition.notify_all();
			// and wait until frame is consumed
			mWaitCondition.wait(lLock);
		}
	}
}
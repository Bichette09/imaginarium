#pragma once

#include "thread_interface.h"

// ros
#include "ros/ros.h"

// std
#include <mutex>
#include <condition_variable>

template <typename FrameType>
class WorkerInterface
{
public:
	WorkerInterface();
	virtual ~WorkerInterface();
	
	virtual bool computeNextResult(FrameType & pRes) = 0;
	
};

template <typename FrameType>
class FrameProcessor : private ThreadInterface
{
public:
	
	
	FrameProcessor(WorkerInterface<FrameType> & pWorker);
	virtual ~FrameProcessor();
	
	bool getNextFrame(FrameType & pFrame);
	
protected:
	virtual void run();
private:
	WorkerInterface<FrameType> & mWorker;
	std::mutex					mMutex;
	std::condition_variable		mWaitCondition;
	FrameType					mTmpFrame;
	FrameType					mNextResult;
	bool						mGotNewResult;
	bool						mQuit;
};

#include "frame_processor.inl"
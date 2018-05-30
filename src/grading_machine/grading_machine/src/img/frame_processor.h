#pragma once

#include "thread_interface.h"
#include "frame.h"

// std
#include <mutex>
#include <condition_variable>


class FrameProcessor : private ThreadInterface
{
public:
	
	class WorkerInterface
	{
	public:
		WorkerInterface();
		virtual ~WorkerInterface();
		
	protected:
		virtual bool computeNextResult(Frame & pRes) = 0;
		
		friend class FrameProcessor;
	};
	
	FrameProcessor(WorkerInterface & pWorker);
	virtual ~FrameProcessor();
	
	bool getNextFrame(Frame & pFrame);
	
protected:
	virtual void run();
private:
	WorkerInterface &			mWorker;
	std::mutex					mMutex;
	std::condition_variable		mWaitCondition;
	Frame						mTmpFrame;
	Frame						mNextResult;
	bool						mGotNewResult;
	bool						mQuit;
};
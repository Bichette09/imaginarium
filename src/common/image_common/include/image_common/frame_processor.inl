
#define USE_THREAD

template <typename FrameType>
WorkerInterface<FrameType>::WorkerInterface()
{
}

template <typename FrameType>
WorkerInterface<FrameType>::~WorkerInterface()
{
}

template <typename FrameType>
FrameProcessor<FrameType>::FrameProcessor(WorkerInterface<FrameType> & pWorker, const std::string & pName)
	: mQuit(false)
	, mGotNewResult(false)
	, mWorker(pWorker)
	, mName(pName)
{
#ifdef USE_THREAD
	startThread();
#endif
}

template <typename FrameType>
FrameProcessor<FrameType>::~FrameProcessor()
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

template <typename FrameType>
bool FrameProcessor<FrameType>::getNextFrame(FrameType & pFrame)
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

template <typename FrameType>
void FrameProcessor<FrameType>::run()
{
	ROS_WARN_STREAM("Start FrameProcessor thread "<<getThreadId()<<" "<<mName);
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
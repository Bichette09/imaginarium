#include "thread_interface.h"

#include <thread>

ThreadInterface::ThreadInterface()
	: mThread(NULL)
{
}

ThreadInterface::~ThreadInterface()
{
	waitThread();
}

void ThreadInterface::startThread()
{
	if(mThread)
		return;
	mThread = new std::thread(Run,this);
}

void ThreadInterface::waitThread()
{
	if(mThread)
	{
		mThread->join();
		delete mThread;
		mThread = NULL;
	}
}

void ThreadInterface::Run(ThreadInterface * pThread)
{
	pThread->run();
}


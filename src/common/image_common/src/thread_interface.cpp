#include "thread_interface.h"

#include <thread>
#include <sstream>

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

std::string ThreadInterface::getThreadId() const
{
	if(!mThread)
		return "no-thread";
	std::stringstream lSS;
	lSS<<mThread->get_id();
	return lSS.str();
}


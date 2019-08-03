#pragma once

// std
#include <string>

namespace std
{
	class thread;
}

class ThreadInterface
{
public:
	
	ThreadInterface();
	virtual ~ThreadInterface();
	
	std::string getThreadId() const;
	
protected:
	void startThread();
	void waitThread();
	
	virtual void run() = 0;
	
private:
	static void Run(ThreadInterface * pThread);
	
	std::thread *	mThread;
};
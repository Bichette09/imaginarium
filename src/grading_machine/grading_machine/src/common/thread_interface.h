#pragma once

// std


namespace std
{
	class thread;
}

class ThreadInterface
{
public:
	
	ThreadInterface();
	virtual ~ThreadInterface();
	
	
protected:
	void startThread();
	void waitThread();
	
	virtual void run() = 0;
	
private:
	static void Run(ThreadInterface * pThread);
	
	std::thread *	mThread;
};
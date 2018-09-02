#include "pause_proxy_frame_provider.h"


PauseProxyFrameProvider::PauseProxyFrameProvider(FrameProvider & pProvider, ros::NodeHandle & pNode)
	: mProvider(pProvider)
	, mIsPaused(false)
	, mNextFrame(false)
	, mNodeHandle(pNode)
	, mStateDeclarator(pNode)
	, mIsFrameOk(false)
{
	mGamePadSubscriber = mNodeHandle.subscribe("/GamePadButtons", 1000, &PauseProxyFrameProvider::onGamepadButton,this);
	mStateDeclarator.setState("video_stream","play");
}

PauseProxyFrameProvider::~PauseProxyFrameProvider()
{
}

void PauseProxyFrameProvider::onGamepadButton(const std_msgs::String::ConstPtr& msg)
{
	if(msg->data.find("|A|") != std::string::npos)
	{
		mIsPaused = true;
		mNextFrame = true;
		mStateDeclarator.setState("video_stream","pause");
	}
	else if(msg->data.find("|X|")  != std::string::npos)
	{
		mIsPaused = false;
		mStateDeclarator.setState("video_stream","play");
	}
}

bool PauseProxyFrameProvider::getNextFrame(FrameInterface & pRes)
{
	if(!mIsPaused || mNextFrame)
	{
		mNextFrame = false;
		if(!mProvider.getNextFrame(pRes))
		{
			mIsFrameOk = false;
			return false;
		}
		mCurrentFrame[0] = pRes.editY();
		mCurrentFrame[1] = pRes.editU();
		mCurrentFrame[2] = pRes.editV();
		mIsFrameOk = true;
	}
	else
	{
		pRes.editY() = mCurrentFrame[0] ;
		pRes.editU() = mCurrentFrame[1] ;
		pRes.editV() = mCurrentFrame[2] ;
	}

	return mIsFrameOk;
	
}

int PauseProxyFrameProvider::getFrameWidth() const
{
	return mProvider.getFrameWidth();
}

int PauseProxyFrameProvider::getFrameHeight() const
{
	return mProvider.getFrameHeight();
}

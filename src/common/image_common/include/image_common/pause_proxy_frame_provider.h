#pragma once


#include "image_common/frame_interface.h"
#include <settings_store/settings_store_client.h>

//ros
#include "ros/ros.h"
#include "std_msgs/String.h"

// opencv
#include "opencv2/opencv.hpp"

// std

/** this frame provider will allow to pause stream or move frame by frame
*/
class PauseProxyFrameProvider : public FrameProvider
{
public:

	PauseProxyFrameProvider(FrameProvider & pProvider, ros::NodeHandle & pNode);
	virtual ~PauseProxyFrameProvider();
	
	virtual bool getNextFrame(FrameInterface & pRes);
	virtual int getFrameWidth() const;
	virtual int getFrameHeight() const;
	
	void onGamepadButton(const std_msgs::String::ConstPtr& msg);
private:
	FrameProvider & mProvider;
	ros::NodeHandle & mNodeHandle;
	settings_store::StateDeclarator mStateDeclarator;
	ros::Subscriber	mGamePadSubscriber;
	cv::Mat			mCurrentFrame[3];
	bool			mIsPaused;
	bool			mNextFrame;
	bool			mIsFrameOk;
};
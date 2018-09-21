#pragma once

// opencv
#include "opencv2/opencv.hpp"

// ros
#include "ros/ros.h"
#include "std_msgs/String.h"

class FrameInterface;
class FrameRecorder
{
public:
	
	FrameRecorder(const std::string & pOutputFolder,int pWidth, int pHeight, int pFps, ros::NodeHandle & pNode);
	~FrameRecorder();
	
	void startRecording();
	void addFrame( FrameInterface & pFrame);
	void endRecording();
	void onGamepadButton(const std_msgs::String::ConstPtr& msg);
private:

	int mWidth;
	int mHeight;
	int mFps;
	const std::string	mOutputFolder;
	cv::VideoWriter * 	mVideoRecorder;
	ros::Subscriber		mGamePadSubscriber;
	cv::Mat				mTmp;
	cv::Mat				mTmp2;
};
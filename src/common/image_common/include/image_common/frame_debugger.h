#pragma once

// opencv
#include "opencv2/opencv.hpp"

// ros
#include "ros/ros.h"
#include <image_transport/image_transport.h>

// std
#include <map>
#include <string>


class FrameDebugger
{
public:
	
	FrameDebugger(ros::NodeHandle & pNodeHandle, const std::string & pFilterString);
	~FrameDebugger();
	
	void setImage(char pName, const cv::Mat & pImage);
	
private:
	ros::NodeHandle &	mNodeHandle;
	const std::string & mFilterString;
	image_transport::ImageTransport mImageTransport;
	
	typedef std::map<char,image_transport::Publisher> tPubMap;
	tPubMap 	mPubs;
	
};
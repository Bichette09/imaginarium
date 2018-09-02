#include "frame_debugger.h"

#include <cv_bridge/cv_bridge.h>


FrameDebugger::FrameDebugger(ros::NodeHandle & pNodeHandle, const std::string & pFilterString)
	: mNodeHandle(pNodeHandle)
	, mImageTransport(mNodeHandle)
	, mFilterString(pFilterString)
{
}

FrameDebugger::~FrameDebugger()
{
}

void FrameDebugger::setImage(char pName, const cv::Mat & pImage)
{
	if(pImage.empty())
		return;
	if(mFilterString.find(pName) == std::string::npos)
	{
		return;
	}
	if(mPubs.find(pName) == mPubs.end())
	{
		mPubs[pName] = mImageTransport.advertise(std::string(1,pName) + "_image", 1);
	}
	sensor_msgs::ImagePtr lMsg = cv_bridge::CvImage(std_msgs::Header(), "mono8", pImage).toImageMsg();
	mPubs[pName].publish(lMsg);
}


// std
#include <iostream>
#include <fstream>
#include <chrono>

// ros
#include "ros/ros.h"
#include "std_msgs/String.h"
 
// settings_store
#include <settings_store/settings_store_client.h>

// opencv
#include "opencv2/opencv.hpp"

#include "image_common/camera_frame_provider.h"
#include "image_common/frame_debugger.h"

#include "thresholding_worker.h"

class LightAndLineDetectorSettings : public settings_store::SettingsBase
{
public:
	LightAndLineDetectorSettings(ros::NodeHandle & pNodeHandle)
		: settings_store::SettingsBase(pNodeHandle)
		, mDebugImgChannels("yuvm")
	{
		registerAttribute<std::string>("ligh_and_line_detector/debug_img_channels",mDebugImgChannels,"which channels should be published for debug ? AYUVM");
		
		declareAndRetrieveSettings();
	}
	
	virtual ~LightAndLineDetectorSettings()
	{
	}
	
	virtual void onParameterChanged(const std::string & pSettingName)
	{
		
	}
	
	ThresholdingWorker::Parameters	mThresholdingParameters;
	std::string						mDebugImgChannels;
};


int main(int argc, char ** argv)
{
	ros::init(argc,argv,"grading_machine_cpp");
	{
		cv::setNumThreads(1);
		
		ros::NodeHandle n;
		
		
		cv::Mat lTmp, lRGB, lYUV;
		
		ros::Rate lLoopRate(1);
		
		LightAndLineDetectorSettings lSettings(n);
		FrameDebugger lFrameDebugger(n,lSettings.mDebugImgChannels);
		
		
		
		CameraFrameProvider lFrameProvider(CameraFrameProvider::Parameters(320*4,240*4,12));
		
		ThresholdingWorker lThresholdingWorker(lFrameProvider,lSettings.mThresholdingParameters);
		FrameProcessor<LightAndLineFrame> lThresholdingThread(lThresholdingWorker);
		
		LightAndLineFrame lFrame;
		
		LightAndLineFrame::tTimestamp lPreviousFrameTs = std::chrono::system_clock::now();
		LightAndLineFrame::tTimestamp lStatStart;
		
		
		while(ros::ok())
		{
			if(lThresholdingThread.getNextFrame(lFrame))
			{
				
				lFrameDebugger.setImage('y',lFrame[LightAndLineFrame::Y]);
				lFrameDebugger.setImage('u',lFrame[LightAndLineFrame::U]);
				lFrameDebugger.setImage('v',lFrame[LightAndLineFrame::V]);
				
			}
			
			ros::spinOnce();

		}
		
	}
	
	return 0;
}


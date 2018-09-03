
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
#include "image_common/video_frame_provider.h"
#include "image_common/pause_proxy_frame_provider.h"
#include "image_common/frame_debugger.h"

#include "thresholding_worker.h"
#include "light_detector.h"

class LightAndLineDetectorSettings : public settings_store::SettingsBase
{
public:
	LightAndLineDetectorSettings(ros::NodeHandle & pNodeHandle)
		: settings_store::SettingsBase(pNodeHandle)
		, mDebugImgChannels("yuvm")
		, mLightTimeWindowSec(10)
	{
		mRedLightParameterString = mThresholdingParameters.mRedLightParameter.getStringFromValues();
		mYellowLightParameterString = mThresholdingParameters.mYellowLightParameter.getStringFromValues();
		mBlueLightParameterString = mThresholdingParameters.mBlueLightParameter.getStringFromValues();
		mLightSearchAreaString = mThresholdingParameters.getLightSearchArea();
		
		registerAttribute<std::string>("ligh_and_line_detector/debug_img_channels",mDebugImgChannels,"which channels should be published for debug ? AYUVM");
		registerAttribute<std::string>("ligh_and_line_detector/lightsearchareapercent",mLightSearchAreaString,"area in percent were light should be searched [xmin xmax ymin ymax]");
		registerAttribute<std::string>("ligh_and_line_detector/red_light",mRedLightParameterString,"Umin Umax Vmin Vmax DownscaleFactor MinPixCountPercentPerDownscaleArea");
		registerAttribute<std::string>("ligh_and_line_detector/yellow_light",mYellowLightParameterString,"Umin Umax Vmin Vmax DownscaleFactor MinPixCountPercentPerDownscaleArea");
		registerAttribute<std::string>("ligh_and_line_detector/blue_light",mBlueLightParameterString,"Umin Umax Vmin Vmax DownscaleFactor MinPixCountPercentPerDownscaleArea");
		
		registerAttribute<uint32_t>("ligh_and_line_detector/light_time_window",mLightTimeWindowSec,1,60,"max duration between red and blue light in sec");
		
		declareAndRetrieveSettings();
	}
	
	virtual ~LightAndLineDetectorSettings()
	{
	}
	
	virtual void onParameterChanged(const std::string & pSettingName)
	{
		if(pSettingName.find("/red_light") != std::string::npos)
		{
			mThresholdingParameters.mRedLightParameter.setValuesFromString(mRedLightParameterString);
		}
		else if(pSettingName.find("/yellow_light") != std::string::npos)
		{
			mThresholdingParameters.mYellowLightParameter.setValuesFromString(mYellowLightParameterString);
		}
		else if(pSettingName.find("/blue_light") != std::string::npos)
		{
			mThresholdingParameters.mBlueLightParameter.setValuesFromString(mBlueLightParameterString);
		}
		else if(pSettingName.find("/lightsearchareapercent") != std::string::npos)
		{
			mThresholdingParameters.setLightSearchArea(mLightSearchAreaString);
		}
	}
	
	ThresholdingWorker::Parameters	mThresholdingParameters;
	std::string						mLightSearchAreaString;
	std::string						mRedLightParameterString;
	std::string						mYellowLightParameterString;
	std::string						mBlueLightParameterString;
	std::string						mDebugImgChannels;
	uint32_t						mLightTimeWindowSec;
};


int main(int argc, char ** argv)
{
	ros::init(argc,argv,"light_and_line_detector");
	{
		cv::setNumThreads(1);
		
		ros::NodeHandle n;
		
		
		ros::Rate lLoopRate(1);
		
		LightAndLineDetectorSettings lSettings(n);
		FrameDebugger lFrameDebugger(n,lSettings.mDebugImgChannels);
		
		bool lEnableLightDetection = true;
		
		settings_store::StateDeclarator lStateDeclarator(n);
		
		//CameraFrameProvider lFrameProviderA(CameraFrameProvider::Parameters(320*4,240*4,12));
		VideoFrameProvider lFrameProviderA(VideoFrameProvider::Parameters(320,240,24,"/home/pi/Untitled Project.avi"));
		PauseProxyFrameProvider lFrameProvider(lFrameProviderA,n);
		
		ThresholdingWorker lThresholdingWorker(lFrameProvider,lSettings.mThresholdingParameters,lEnableLightDetection);
		FrameProcessor<LightAndLineFrame> lThresholdingThread(lThresholdingWorker);
		
		LightAndLineFrame lFrame;
		
		LightAndLineFrame::tTimestamp lPreviousFrameTs = std::chrono::system_clock::now();
		LightAndLineFrame::tTimestamp lStatStart;
		
		LightDetector lLightDetector(50,50);
		cv::Mat lTmp;
		
		while(ros::ok())
		{
			if(lThresholdingThread.getNextFrame(lFrame))
			{
				if(lEnableLightDetection)
				{
					lLightDetector.addNewFrame(lFrame);
					if(lSettings.mDebugImgChannels.find("D") != std::string::npos)
					{
						lLightDetector.createDebugImg(lFrame[LightAndLineFrame::Debug2],lSettings.mLightTimeWindowSec);
					}
					if(lLightDetector.detectLightSequence(lSettings.mLightTimeWindowSec))
					{
						lLightDetector.clearDetector();
						ROS_WARN_STREAM("GOGOGOGO !");
					}
				}
				
				if(lSettings.mDebugImgChannels.find("d") != std::string::npos)
				{
					std::vector<cv::Mat> lToMerge;
					lToMerge.push_back(lFrame[LightAndLineFrame::Y]);
					lToMerge.push_back(lFrame[LightAndLineFrame::U]);
					lToMerge.push_back(lFrame[LightAndLineFrame::V]);
					cv::merge(lToMerge, lTmp);
					cvtColor(lTmp, lFrame[LightAndLineFrame::Debug], cv::COLOR_YUV2BGR);
					cv::rectangle(lFrame[LightAndLineFrame::Debug],lFrame.getLightSearchArea(),cv::Scalar(255,255,255),2);
					
					for(int i = 0 ; i < lFrame[LightAndLineFrame::LC_Red].size() ; ++i)
						cv::rectangle(lFrame[LightAndLineFrame::Debug],lFrame[LightAndLineFrame::LC_Red][i],cv::Scalar(0,0,255),2);
					for(int i = 0 ; i < lFrame[LightAndLineFrame::LC_Yellow].size() ; ++i)
						cv::rectangle(lFrame[LightAndLineFrame::Debug],lFrame[LightAndLineFrame::LC_Yellow][i],cv::Scalar(0,255,255),2);
					for(int i = 0 ; i < lFrame[LightAndLineFrame::LC_Blue].size() ; ++i)
						cv::rectangle(lFrame[LightAndLineFrame::Debug],lFrame[LightAndLineFrame::LC_Blue][i],cv::Scalar(255,0,0),2);
				}
				
				
				

				lFrameDebugger.setImage('y',lFrame[LightAndLineFrame::Y]);
				lFrameDebugger.setImage('u',lFrame[LightAndLineFrame::U]);
				lFrameDebugger.setImage('v',lFrame[LightAndLineFrame::V]);
				lFrameDebugger.setImage('d',lFrame[LightAndLineFrame::Debug]);
				lFrameDebugger.setImage('D',lFrame[LightAndLineFrame::Debug2]);
			}
			
			ros::spinOnce();

		}
		
	}
	
	return 0;
}


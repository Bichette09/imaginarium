
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
#include "emaginarium_common/LightAndLineDetectionStats.h"

class LightAndLineDetectorSettings : public settings_store::SettingsBase
{
public:
	LightAndLineDetectorSettings(ros::NodeHandle & pNodeHandle)
		: settings_store::SettingsBase(pNodeHandle)
		, mDebugImgChannels("yuvdDL")
		, mLightTimeWindowSec(10)
	{
		mRedLightParameterString = mThresholdingParameters.mRedLightParameter.getStringFromValues();
		mYellowLightParameterString = mThresholdingParameters.mYellowLightParameter.getStringFromValues();
		mBlueLightParameterString = mThresholdingParameters.mBlueLightParameter.getStringFromValues();
		mLightSearchAreaString = mThresholdingParameters.mLightSearchArea.getStringFromValues();
		mLineSearchAreaString = mThresholdingParameters.mLineSearchArea.getStringFromValues();
		mLineColorParameterString = mThresholdingParameters.mLineColorParameter.getStringFromValues();
		
		registerAttribute<std::string>("light_and_line_detector/debug_img_channels",mDebugImgChannels,"which channels should be published for debug ? yuvdDL");
		registerAttribute<std::string>("light/searchareapercent",mLightSearchAreaString,"area in percent were light should be searched [xmin xmax ymin ymax]");
		registerAttribute<std::string>("light/red",mRedLightParameterString,"Ymin Ymax Umin Umax Vmin Vmax | DownscaleFactor MinPixCountPercentPerDownscaleArea");
		registerAttribute<std::string>("light/yellow",mYellowLightParameterString,"Ymin Ymax Umin Umax Vmin Vmax | DownscaleFactor MinPixCountPercentPerDownscaleArea");
		registerAttribute<std::string>("light/blue",mBlueLightParameterString,"Ymin Ymax Umin Umax Vmin Vmax | DownscaleFactor MinPixCountPercentPerDownscaleArea");
		registerAttribute<uint32_t>("light/time_window",mLightTimeWindowSec,1,60,"max duration between red and blue light in sec");
		
		registerAttribute<std::string>("line/searchareapercent",mLineSearchAreaString,"area in percent were line should be searched [xmin xmax ymin ymax]");
		registerAttribute<std::string>("line/color",mLineColorParameterString,"Ymin Ymax Umin Umax Vmin Vmax | CanniTh HoughTh minLineLen maxLineGab");
		declareAndRetrieveSettings();
	}
	
	virtual ~LightAndLineDetectorSettings()
	{
	}
	
	virtual void onParameterChanged(const std::string & pSettingName)
	{
		if(pSettingName.find("light/red") != std::string::npos)
		{
			mThresholdingParameters.mRedLightParameter.setValuesFromString(mRedLightParameterString);
		}
		else if(pSettingName.find("light/yellow") != std::string::npos)
		{
			mThresholdingParameters.mYellowLightParameter.setValuesFromString(mYellowLightParameterString);
		}
		else if(pSettingName.find("light/blue") != std::string::npos)
		{
			mThresholdingParameters.mBlueLightParameter.setValuesFromString(mBlueLightParameterString);
		}
		else if(pSettingName.find("light/searchareapercent") != std::string::npos)
		{
			mThresholdingParameters.mLightSearchArea.setValuesFromString(mLightSearchAreaString);
		}
		else if(pSettingName.find("line/searchareapercent") != std::string::npos)
		{
			mThresholdingParameters.mLineSearchArea.setValuesFromString(mLineSearchAreaString);
		}
		else if(pSettingName.find("line/color") != std::string::npos)
		{
			mThresholdingParameters.mLineColorParameter.setValuesFromString(mLineColorParameterString);
		}
	}
	
	ThresholdingWorker::Parameters	mThresholdingParameters;
	std::string						mLightSearchAreaString;
	std::string						mRedLightParameterString;
	std::string						mYellowLightParameterString;
	std::string						mBlueLightParameterString;
	std::string						mDebugImgChannels;
	std::string						mLineSearchAreaString;
	std::string						mLineColorParameterString;
	uint32_t						mLightTimeWindowSec;
};


int main(int argc, char ** argv)
{
	ros::init(argc,argv,"light_and_line_detector");
	{
		cv::setNumThreads(1);
		
		ros::NodeHandle n;
		
		ros::Publisher lMsgPublisher = n.advertise<std_msgs::String>("light_and_line_detector/event", 10);
		ros::Publisher lPubStat = n.advertise<emaginarium_common::LightAndLineDetectionStats>("light_and_line_detector/stats",1);
		
		ros::Rate lLoopRate(1);
		
		LightAndLineDetectorSettings lSettings(n);
		FrameDebugger lFrameDebugger(n,lSettings.mDebugImgChannels);
		
		bool lEnableLightDetection = true;
		
		settings_store::StateDeclarator lStateDeclarator(n);
		
#if 1
		CameraFrameProvider lFrameProvider(CameraFrameProvider::Parameters(320*2,240*2,24));
#else
		VideoFrameProvider lFrameProviderA(VideoFrameProvider::Parameters(640,480,24,"/home/pi/Untitled Project.avi"));
		PauseProxyFrameProvider lFrameProvider(lFrameProviderA,n);
#endif

		ThresholdingWorker lThresholdingWorker(lFrameProvider,lSettings.mThresholdingParameters,lEnableLightDetection);
		FrameProcessor<LightAndLineFrame> lThresholdingThread(lThresholdingWorker);
		
		LightAndLineFrame lFrame;
		
		LightDetector lLightDetector(50,50);
		cv::Mat lTmp;
		
		LightAndLineFrame::tTimestamp lPreviousFrameTs = std::chrono::system_clock::now();
		LightAndLineFrame::tTimestamp lStatStart;
		emaginarium_common::LightAndLineDetectionStats lNextStat;
		int lStatCptr = 0;
		
		while(ros::ok())
		{
			if(lThresholdingThread.getNextFrame(lFrame))
			{
				lFrame.setTimestamp(LightAndLineFrame::F_LightAnalyzeStart);
				if(lEnableLightDetection)
				{
					
					lLightDetector.addNewFrame(lFrame);
					if(lSettings.mDebugImgChannels.find("L") != std::string::npos)
					{
						lLightDetector.createDebugImg(lFrame[LightAndLineFrame::LightStatus],lSettings.mLightTimeWindowSec);
					}
					if(lLightDetector.detectLightSequence(lSettings.mLightTimeWindowSec))
					{
						lLightDetector.clearDetector();
						std_msgs::String lMsg;
						lMsg.data = "start_light_sequence_detected";
						lMsgPublisher.publish(lMsg);
						ROS_WARN_STREAM("GOGOGOGO !");
					}
				}
				lFrame.setTimestamp(LightAndLineFrame::F_LightAnalyzeDone);
				
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
					
					cv::rectangle(lFrame[LightAndLineFrame::Debug],lFrame.getLineSearchArea(),cv::Scalar(128,128,128),2);
					for(int i = 0 ; i < lFrame.getLines().size() ; ++i)
					{
						cv::line(lFrame[LightAndLineFrame::Debug],
							cv::Point(lFrame.getLines()[i][0],lFrame.getLines()[i][1]),
							cv::Point(lFrame.getLines()[i][2],lFrame.getLines()[i][3]),
							cv::Scalar(255,0,0),2);
					}
				}
				
				
				

				lFrameDebugger.setImage('y',lFrame[LightAndLineFrame::Y]);
				lFrameDebugger.setImage('u',lFrame[LightAndLineFrame::U]);
				lFrameDebugger.setImage('v',lFrame[LightAndLineFrame::V]);
				lFrameDebugger.setImage('d',lFrame[LightAndLineFrame::Debug]);
				lFrameDebugger.setImage('D',lFrame[LightAndLineFrame::Debug2]);
				lFrameDebugger.setImage('L',lFrame[LightAndLineFrame::LightStatus]);
				
				// update stats
				{
					if(lStatCptr == 0)
					{
						lStatStart = std::chrono::system_clock::now();
						lNextStat = emaginarium_common::LightAndLineDetectionStats();
					}
					++lStatCptr;
					
					LightAndLineFrame::tTimestamp lNow = std::chrono::system_clock::now();
					lNextStat.fps += 1.f/std::chrono::duration<float>(lNow - lPreviousFrameTs).count();
					lPreviousFrameTs = lNow;
					lNextStat.latency += std::chrono::duration<float,std::milli>(lNow - lFrame[LightAndLineFrame::F_GrabDone]).count();
					lNextStat.lightthresholding += std::chrono::duration<float,std::milli>(lFrame[LightAndLineFrame::F_LightThresholdingDone] - lFrame[LightAndLineFrame::F_LightThresholdingStart]).count();
					lNextStat.lightanalyze += std::chrono::duration<float,std::milli>(lFrame[LightAndLineFrame::F_LightAnalyzeDone] - lFrame[LightAndLineFrame::F_LightAnalyzeStart]).count();
					lNextStat.linethresholding += std::chrono::duration<float,std::milli>(lFrame[LightAndLineFrame::F_LineThresholdingDone] - lFrame[LightAndLineFrame::F_LineThresholdingStart]).count();
					if(std::chrono::duration<float>(lNow - lStatStart).count() > 0.25)
					{
						lNextStat.fps /= lStatCptr;
						lNextStat.latency /= lStatCptr;
						lNextStat.lightthresholding /= lStatCptr;
						lNextStat.lightanalyze /= lStatCptr;
						lNextStat.linethresholding /= lStatCptr;
						lStatCptr = 0;
						lPubStat.publish(lNextStat);
						
					}
				}
			}
			
			ros::spinOnce();

		}
		
	}
	
	return 0;
}


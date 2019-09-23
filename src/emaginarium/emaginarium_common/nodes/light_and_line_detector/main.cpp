
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
#include "image_common/frame_recorder.h"

#include "thresholding_worker.h"
#include "light_detector.h"
#include "line_detector.h"
#include "emaginarium_common/LightAndLineDetectionStats.h"

class LightAndLineDetectorSettings : public settings_store::SettingsBase
{
public:
	LightAndLineDetectorSettings(ros::NodeHandle & pNodeHandle)
		: settings_store::SettingsBase(pNodeHandle)
		, mDebugImgChannels("yuvdDLl")
		, mLightTimeWindowMSec(10000)
		, mLineTimeWindowMSec(10000)
		, mLightDetection(true)
		, mLineDetection(true)
		, mGlobalEnable(false)
	{
		mRedLightParameterString = mThresholdingParameters.mRedLightParameter.getStringFromValues();
		mYellowLightParameterString = mThresholdingParameters.mYellowLightParameter.getStringFromValues();
		mBlueLightParameterString = mThresholdingParameters.mBlueLightParameter.getStringFromValues();
		mLightSearchAreaString = mThresholdingParameters.mLightSearchArea.getStringFromValues();
		mLineSearchAreaString = mThresholdingParameters.mLineSearchArea.getStringFromValues();
		mRedLineColorParameterString = mThresholdingParameters.mRedLineColorParameter.getStringFromValues();
		mGreenLineColorParameterString = mThresholdingParameters.mGreenLineColorParameter.getStringFromValues();
		mRedLineHoughParameterString = mThresholdingParameters.mRedLineHough.getStringFromValues();
		mGreenLineHoughParameterString = mThresholdingParameters.mGreenLineHough.getStringFromValues();
		
		const char * lColorDesc = "Ymin Ymax Umin Umax Vmin Vmax | DebugMask | DownscaleFactor MinPixCountPercentPerDownscaleArea";
		
		registerAttribute<std::string>("light_and_line_detector/debug_img_channels",mDebugImgChannels,"which channels should be published for debug ? yuvcodDL");
		registerAttribute<std::string>("light/searchareapercent",mLightSearchAreaString,"area in percent were light should be searched [xmin xmax ymin ymax]");
		registerAttribute<std::string>("light/red",mRedLightParameterString,lColorDesc);
		registerAttribute<std::string>("light/yellow",mYellowLightParameterString,lColorDesc);
		registerAttribute<std::string>("light/blue",mBlueLightParameterString,lColorDesc);
		registerAttribute<uint32_t>("light/time_window",mLightTimeWindowMSec,1,60,"max duration between red and blue light in msec");
		
		registerAttribute<std::string>("line/searchareapercent",mLineSearchAreaString,"area in percent were line should be searched [xmin xmax ymin ymax]");
		registerAttribute<std::string>("line/redcolor",mRedLineColorParameterString,lColorDesc);
		registerAttribute<std::string>("line/greencolor",mGreenLineColorParameterString,lColorDesc);
		registerAttribute<std::string>("line/redhough",mRedLineHoughParameterString,"HoughTh minLineLen maxLineGab");
		registerAttribute<std::string>("line/greenhough",mGreenLineHoughParameterString,"HoughTh minLineLen maxLineGab");
		registerAttribute<uint32_t>("line/time_window",mLineTimeWindowMSec,1,60,"duration during which we keep info in detection buffer in msec");
		
		registerAttribute<int32_t>("line/cannyth",mThresholdingParameters.mCannyThreshold,0,255,"Canny threshold");
		
		registerAttribute<bool>("img/lightdetection",mLightDetection,"Enable light detection");
		registerAttribute<bool>("img/linedetection",mLineDetection,"Enable line detection");

		registerAttribute<std::string>("command/mode",mCommandMode,"Robot mode, nominal, manuel ou dlvv");


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
		else if(pSettingName.find("line/redcolor") != std::string::npos)
		{
			mThresholdingParameters.mRedLineColorParameter.setValuesFromString(mRedLineColorParameterString);
		}
		else if(pSettingName.find("line/greencolor") != std::string::npos)
		{
			mThresholdingParameters.mGreenLineColorParameter.setValuesFromString(mGreenLineColorParameterString);
		}
		else if(pSettingName.find("line/redhough") != std::string::npos)
		{
			mThresholdingParameters.mRedLineHough.setValuesFromString(mRedLineHoughParameterString);
		}
		else if(pSettingName.find("line/greenhough") != std::string::npos)
		{
			mThresholdingParameters.mGreenLineHough.setValuesFromString(mGreenLineHoughParameterString);
		}
		else if(pSettingName.find("command/mode") != std::string::npos)
		{
			mGlobalEnable = mCommandMode == "dlvv";
			if(!mGlobalEnable)
			{
				ROS_WARN_STREAM("Light detection disabled (command/mode != dlvv)");
			}
			else
			{
				ROS_WARN_STREAM("Light detection enabled (command/mode == dlvv)");
			}
		}
	}
	
	ThresholdingWorker::Parameters	mThresholdingParameters;
	
	std::string						mLightSearchAreaString;
	std::string						mRedLightParameterString;
	std::string						mYellowLightParameterString;
	std::string						mBlueLightParameterString;
	std::string						mDebugImgChannels;
	
	std::string						mLineSearchAreaString;
	std::string						mRedLineColorParameterString;
	std::string						mGreenLineColorParameterString;
	std::string						mRedLineHoughParameterString;
	std::string						mGreenLineHoughParameterString;
	
	std::string						mCommandMode;
	
	uint32_t						mLightTimeWindowMSec;
	uint32_t						mLineTimeWindowMSec;
	
	bool							mLightDetection;
	bool							mLineDetection;
	bool							mGlobalEnable;
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
		
		
		settings_store::StateDeclarator lStateDeclarator(n);
//#define USE_CAM
#ifdef USE_CAM
		CameraFrameProvider lFrameProvider(CameraFrameProvider::Parameters(320*2,240*2,12));
		
		FrameRecorder lVideoRecorder("/home/pi",lFrameProvider.mParameters.mHalfWidth,lFrameProvider.mParameters.mHalfHeight,lFrameProvider.mParameters.mFps,n);
#else
		VideoFrameProvider lFrameProviderA(VideoFrameProvider::Parameters(640,480,12,"/home/pi/vid_20.avi"));
		PauseProxyFrameProvider lFrameProvider(lFrameProviderA,n);
#endif

		bool lEnableLightDetection = true;
		bool lEnableLineDetection = true;

		ThresholdingWorker lThresholdingWorkerA(&lFrameProvider,NULL,lSettings.mThresholdingParameters,lEnableLightDetection);
		ThresholdingWorker lThresholdingWorkerB(NULL,&lThresholdingWorkerA,lSettings.mThresholdingParameters,lEnableLightDetection);
		FrameProcessor<LightAndLineFrame> lFinalThread(lThresholdingWorkerB,"final");
		
		
		LightAndLineFrame lFrame;
		
		LightDetector lLightDetector(50,50);
		LineDetector lLineDetector(25);
		cv::Mat lTmp;
		
		LightAndLineFrame::tTimestamp lPreviousFrameTs = std::chrono::system_clock::now();
		LightAndLineFrame::tTimestamp lStatStart;
		emaginarium_common::LightAndLineDetectionStats lNextStat;
		int lStatCptr = 0;
		
		bool lSleep = false;
		while(ros::ok())
		{
			lSleep = true;
			// update light and line detection flag
			lEnableLightDetection = lSettings.mLightDetection && lSettings.mGlobalEnable;
			lEnableLineDetection = lSettings.mLineDetection && lSettings.mGlobalEnable;
			
			lFrameProvider.setEnabled(lSettings.mGlobalEnable);
			
			lStateDeclarator.setState<bool>("img/lightdetection",lEnableLightDetection);
			lStateDeclarator.setState<bool>("img/linedetection",lEnableLineDetection);
			
			if(!lSettings.mGlobalEnable)
				goto endLoop;
			
			if(!lFinalThread.getNextFrame(lFrame))
				goto endLoop;
			
			lSleep = false;
			
			{
				lFrame.setTimestamp(LightAndLineFrame::F_LightAnalyzeStart);
				if(lEnableLightDetection)
				{
					
					lLightDetector.addNewFrame(lFrame);
					if(lSettings.mDebugImgChannels.find("L") != std::string::npos)
					{
						lLightDetector.createDebugImg(lFrame[LightAndLineFrame::LightStatus],lSettings.mLightTimeWindowMSec);
					}
					if(lLightDetector.detectLightSequence(lSettings.mLightTimeWindowMSec))
					{
						lLightDetector.clearDetector();
						std_msgs::String lMsg;
						lMsg.data = "start_light_sequence_detected";
						lMsgPublisher.publish(lMsg);
						ROS_WARN_STREAM("GOGOGOGO !");
					}
				}
				lFrame.setTimestamp(LightAndLineFrame::F_LightAnalyzeDone);
				
				
				if(lEnableLineDetection)
				{
					
					lLineDetector.addNewFrame(lFrame,lSettings.mLineTimeWindowMSec);
					
					if(lLineDetector.detectLine(LightAndLineFrame::LT_Green))
					{
						lLineDetector.clearDetector();
						std_msgs::String lMsg;
						lMsg.data = "start_line_detected";
						lMsgPublisher.publish(lMsg);
						ROS_WARN_STREAM("START LINE !");
					}
					else if(lLineDetector.detectLine(LightAndLineFrame::LT_Red))
					{
						lLineDetector.clearDetector();
						std_msgs::String lMsg;
						lMsg.data = "finish_line_detected";
						lMsgPublisher.publish(lMsg);
						ROS_WARN_STREAM("FINISH LINE !");
					}
					
					if(lSettings.mDebugImgChannels.find("l") != std::string::npos)
					{
						lLineDetector.createDebugImg(lFrame[LightAndLineFrame::LineStatus],lSettings.mLineTimeWindowMSec);
					}
					
				}
				
				
#ifdef USE_CAM
				lVideoRecorder.addFrame(lFrame);
#endif
				if(lSettings.mDebugImgChannels.find("o") != std::string::npos)
				{
					std::vector<cv::Mat> lToMerge;
					lToMerge.push_back(lFrame[LightAndLineFrame::Y]);
					lToMerge.push_back(lFrame[LightAndLineFrame::U]);
					lToMerge.push_back(lFrame[LightAndLineFrame::V]);
					cv::merge(lToMerge, lTmp);
					cvtColor(lTmp, lFrame[LightAndLineFrame::Overlay], cv::COLOR_YUV2BGR);
					
					cv::rectangle(lFrame[LightAndLineFrame::Overlay],lFrame.getLightSearchArea(),cv::Scalar(255,255,255),2);
					
					for(int i = 0 ; i < lFrame[LightAndLineFrame::LC_Red].size() ; ++i)
						cv::rectangle(lFrame[LightAndLineFrame::Overlay],lFrame[LightAndLineFrame::LC_Red][i],cv::Scalar(0,0,255),2);
					for(int i = 0 ; i < lFrame[LightAndLineFrame::LC_Yellow].size() ; ++i)
						cv::rectangle(lFrame[LightAndLineFrame::Overlay],lFrame[LightAndLineFrame::LC_Yellow][i],cv::Scalar(0,255,255),2);
					for(int i = 0 ; i < lFrame[LightAndLineFrame::LC_Blue].size() ; ++i)
						cv::rectangle(lFrame[LightAndLineFrame::Overlay],lFrame[LightAndLineFrame::LC_Blue][i],cv::Scalar(255,0,0),2);
					for(int i = 0 ; i < lFrame[LightAndLineFrame::LC_LineGreen].size() ; ++i)
						cv::rectangle(lFrame[LightAndLineFrame::Overlay],lFrame[LightAndLineFrame::LC_LineGreen][i],cv::Scalar(0,255,0),2);
					for(int i = 0 ; i < lFrame[LightAndLineFrame::LC_LineRed].size() ; ++i)
						cv::rectangle(lFrame[LightAndLineFrame::Overlay],lFrame[LightAndLineFrame::LC_LineRed][i],cv::Scalar(0,0,255),2);
					
					cv::rectangle(lFrame[LightAndLineFrame::Overlay],lFrame.getLineSearchArea(),cv::Scalar(128,128,128),2);
					for(int i = 0 ; i < lFrame[LightAndLineFrame::LT_Red].size() ; ++i)
					{
						cv::line(lFrame[LightAndLineFrame::Overlay],
							cv::Point(lFrame[LightAndLineFrame::LT_Red][i][0],lFrame[LightAndLineFrame::LT_Red][i][1]),
							cv::Point(lFrame[LightAndLineFrame::LT_Red][i][2],lFrame[LightAndLineFrame::LT_Red][i][3]),
							cv::Scalar(0,0,255),2);
					}
					for(int i = 0 ; i < lFrame[LightAndLineFrame::LT_Green].size() ; ++i)
					{
						cv::line(lFrame[LightAndLineFrame::Overlay],
							cv::Point(lFrame[LightAndLineFrame::LT_Green][i][0],lFrame[LightAndLineFrame::LT_Green][i][1]),
							cv::Point(lFrame[LightAndLineFrame::LT_Green][i][2],lFrame[LightAndLineFrame::LT_Green][i][3]),
							cv::Scalar(0,128,0),2);
					}
				}
				
				
				

				lFrameDebugger.setImage('y',lFrame[LightAndLineFrame::Y]);
				lFrameDebugger.setImage('u',lFrame[LightAndLineFrame::U]);
				lFrameDebugger.setImage('v',lFrame[LightAndLineFrame::V]);
				lFrameDebugger.setImage('c',lFrame[LightAndLineFrame::CannyEdges]);
				lFrameDebugger.setImage('o',lFrame[LightAndLineFrame::Overlay]);
				lFrameDebugger.setImage('d',lFrame[LightAndLineFrame::Debug]);
				lFrameDebugger.setImage('D',lFrame[LightAndLineFrame::Debug2]);
				lFrameDebugger.setImage('L',lFrame[LightAndLineFrame::LightStatus]);
				lFrameDebugger.setImage('l',lFrame[LightAndLineFrame::LineStatus]);
				
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
					lNextStat.linecanyfilter += std::chrono::duration<float,std::milli>(lFrame[LightAndLineFrame::F_LineCanyFilterDone] - lFrame[LightAndLineFrame::F_LineCanyFilterStart]).count();
					lNextStat.linehoughfilter += std::chrono::duration<float,std::milli>(lFrame[LightAndLineFrame::F_LineHoughFilterDone] - lFrame[LightAndLineFrame::F_LineHoughFilterStart]).count();
					if(std::chrono::duration<float>(lNow - lStatStart).count() > 0.25)
					{
						lNextStat.fps /= lStatCptr;
						lNextStat.latency /= lStatCptr;
						lNextStat.lightthresholding /= lStatCptr;
						lNextStat.lightanalyze /= lStatCptr;
						lNextStat.linecanyfilter /= lStatCptr;
						lNextStat.linehoughfilter /= lStatCptr;
						lStatCptr = 0;
						lPubStat.publish(lNextStat);
						
					}
				}
			}
			
			endLoop:
			
			if(lSleep)
				usleep(250000);
			
			ros::spinOnce();

		}
		
	}
	return 0;
}


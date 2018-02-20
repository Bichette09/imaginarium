
// std
#include <iostream>
#include <fstream>
#include <chrono>

// ros
#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include "std_msgs/String.h"
 
// settings_store
#include <settings_store/settings_store_client.h>

// opencv
#include "opencv2/opencv.hpp"

#include "grading_machine/ImgProcessorStat.h"
#include "camera_thread.h"
#include "filter_thread.h"

class GradingMachineCppSettings : public settings_store::SettingsBase
{
public:
	GradingMachineCppSettings(ros::NodeHandle & pNodeHandle)
		: settings_store::SettingsBase(pNodeHandle)
		, mDebugImgChannels("yuvm")
	{
		registerAttribute<int>("grading_machine/filter_th_U",mFilterParameters.mThresholdU,-1,255);
		registerAttribute<int>("grading_machine/filter_th_V",mFilterParameters.mThresholdV,-1,255);
		registerAttribute<bool>("grading_machine/filter_gaussian",mFilterParameters.mGaussian);
		registerAttribute<bool>("grading_machine/filter_dilate",mFilterParameters.mDilate);
		registerAttribute<bool>("grading_machine/filter_erode",mFilterParameters.mErode);
		registerAttribute<float>("grading_machine/exclusion_percent_top",mFilterParameters.mExclusionZoneTopPercent,0.,0.25);
		registerAttribute<float>("grading_machine/exclusion_percent_bottom",mFilterParameters.mExclusionZoneBottomPercent,0.,0.25);
		registerAttribute<std::string>("grading_machine/debug_img_channels",mDebugImgChannels);
		
		declareAndRetrieveSettings();
	}
	
	virtual ~GradingMachineCppSettings()
	{
	}
	
	FilterThread::Parameters	mFilterParameters;
	std::string					mDebugImgChannels;
};


int main(int argc, char ** argv)
{
	ros::init(argc,argv,"grading_machine_cpp");
	{
		cv::setNumThreads(4);
		
		ros::NodeHandle n;
		image_transport::ImageTransport it(n);
		image_transport::Publisher lPubY = it.advertise("Y_image", 1);
		image_transport::Publisher lPubU = it.advertise("U_image", 1);
		image_transport::Publisher lPubV = it.advertise("V_image", 1);
		image_transport::Publisher lPubM = it.advertise("M_image", 1);
		ros::Publisher lPubStat = n.advertise<grading_machine::ImgProcessorStat>("grading_machine/ImgProcessorStat",10);
		
		ros::Rate lLoopRate(1);
		
		GradingMachineCppSettings lSettings(n);
		
		
		
		CameraThread lCameraThread(CameraThread::Parameters(1600,1680,25));
		FilterThread lFilterThread(lCameraThread,lSettings.mFilterParameters);
		lCameraThread.waitForCapture();

		GrabbedFrame lFrame;
		
		GrabbedFrame::tTimestamp lPreviousFrameTs = std::chrono::system_clock::now();
		GrabbedFrame::tTimestamp lStatStart;
		grading_machine::ImgProcessorStat lNextStat;
		int lStatCptr = 0;
		
		while(ros::ok())
		{
			if(lFilterThread.getNextFrame(lFrame))
			{
				if(lStatCptr == 0)
				{
					lStatStart = std::chrono::system_clock::now();
					lNextStat = grading_machine::ImgProcessorStat();
				}
				++lStatCptr;
				
				GrabbedFrame::tTimestamp lNow = std::chrono::system_clock::now();
				lNextStat.fps += 1.f/std::chrono::duration<float>(lNow - lPreviousFrameTs).count();
				lPreviousFrameTs = lNow;
				lNextStat.latency += std::chrono::duration<float>(lNow - lFrame[GrabbedFrame::F_GrabDone]).count();
				lNextStat.filter += std::chrono::duration<float>(lFrame[GrabbedFrame::F_FilterDone] - lFrame[GrabbedFrame::F_FilterStart]).count();
				
				if(std::chrono::duration<float>(lNow - lStatStart).count() > 1.)
				{
					lNextStat.fps /= lStatCptr;
					lNextStat.latency /= lStatCptr;
					lNextStat.filter /= lStatCptr;
					lNextStat.areaextraction /= lStatCptr;
					lStatCptr = 0;
					lPubStat.publish(lNextStat);
					
				}
				
				if( lSettings.mDebugImgChannels.find("y") != std::string::npos)
				{
					sensor_msgs::ImagePtr lMsg = cv_bridge::CvImage(std_msgs::Header(), "mono8", lFrame[GrabbedFrame::Y]).toImageMsg();
					lPubY.publish(lMsg);
				}
				if( lSettings.mDebugImgChannels.find("u") != std::string::npos)
				{
					sensor_msgs::ImagePtr lMsg = cv_bridge::CvImage(std_msgs::Header(), "mono8", lFrame[GrabbedFrame::U]).toImageMsg();
					lPubU.publish(lMsg);
				}
				if( lSettings.mDebugImgChannels.find("v") != std::string::npos)
				{
					sensor_msgs::ImagePtr lMsg = cv_bridge::CvImage(std_msgs::Header(), "mono8", lFrame[GrabbedFrame::V]).toImageMsg();
					lPubV.publish(lMsg);
				}
				if( lSettings.mDebugImgChannels.find("m") != std::string::npos)
				{
					sensor_msgs::ImagePtr lMsg = cv_bridge::CvImage(std_msgs::Header(), "mono8", lFrame[GrabbedFrame::BackgroundMask]).toImageMsg();
					lPubM.publish(lMsg);
				}
				
			}
			
			ros::spinOnce();

		}
		
	}
	
	return 0;
}


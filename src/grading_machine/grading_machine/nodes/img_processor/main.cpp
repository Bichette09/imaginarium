
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

#include "camera_thread.h"
#include "filter_thread.h"

class GradingMachineCppSettings : public settings_store::SettingsBase
{
public:
	GradingMachineCppSettings(ros::NodeHandle & pNodeHandle)
		: settings_store::SettingsBase(pNodeHandle)
		, mUpdateIntervalSec(1.)
		
	{
		registerAttribute<float>("grading_machine/update_period",mUpdateIntervalSec,0.5,10);
		registerAttribute<int>("grading_machine/filter_th_U",mFilterParameters.mThresholdU,-1,255);
		registerAttribute<int>("grading_machine/filter_th_V",mFilterParameters.mThresholdV,-1,255);
		registerAttribute<bool>("grading_machine/filter_gaussian",mFilterParameters.mGaussian);
		registerAttribute<bool>("grading_machine/filter_dilate",mFilterParameters.mDilate);
		registerAttribute<bool>("grading_machine/filter_erode",mFilterParameters.mErode);
		registerAttribute<float>("grading_machine/exclusion_percent_top",mFilterParameters.mExclusionZoneTopPercent,0.,0.25);
		registerAttribute<float>("grading_machine/exclusion_percent_bottom",mFilterParameters.mExclusionZoneBottomPercent,0.,0.25);
		
		declareAndRetrieveSettings();
	}
	
	virtual ~GradingMachineCppSettings()
	{
	}
	
	float mUpdateIntervalSec;
	FilterThread::Parameters	mFilterParameters;
};


int main(int argc, char ** argv)
{
	ros::init(argc,argv,"grading_machine_cpp");
	{
		cv::setNumThreads(4);
		
		ros::NodeHandle n;
		image_transport::ImageTransport it(n);
		image_transport::Publisher pubY = it.advertise("Y_image", 1);
		image_transport::Publisher pubU = it.advertise("U_image", 1);
		image_transport::Publisher pubV = it.advertise("V_image", 1);
		
		ros::Rate lLoopRate(4);
		
		GradingMachineCppSettings lSettings(n);
		
		
		
		CameraThread lCameraThread(CameraThread::Parameters(1600,1680,25));
		FilterThread lFilterThread(lCameraThread,lSettings.mFilterParameters);
		lCameraThread.waitForCapture();

		GrabbedFrame lFrame;
		
		int lCptr = 0;
		std::chrono::time_point<std::chrono::system_clock> lStartTs = std::chrono::system_clock::now();
		GrabbedFrame::tTimestamp lPreviousTs;
		float lLatencyAvg = 0.;
		float lFpsAvg = 0.;
		while(ros::ok())
		{
			if(lFilterThread.getNextFrame(lFrame))
			{
				
				++lCptr;
				float lLatency, lFps;
				GrabbedFrame::ComputeLatencyAndFps(lPreviousTs,lFrame.getTimestamp(),lLatency,lFps);
				lLatencyAvg += lLatency;
				lFpsAvg += lFps;
				// std::cout<<lLatency<<" "<<lFps<<std::endl;
				lPreviousTs = lFrame.getTimestamp();
				
				// sensor_msgs::ImagePtr msgU = cv_bridge::CvImage(std_msgs::Header(), "mono8", lFrame[GrabbedFrame::U]).toImageMsg();
				// pubU.publish(msgU);
				
				// sensor_msgs::ImagePtr msgV = cv_bridge::CvImage(std_msgs::Header(), "mono8", lFrame[GrabbedFrame::V]).toImageMsg();
				// pubV.publish(msgV);
				
				sensor_msgs::ImagePtr msgY = cv_bridge::CvImage(std_msgs::Header(), "mono8", lFrame[GrabbedFrame::BackgroundMask]).toImageMsg();
				pubY.publish(msgY);
			}
			
			ros::spinOnce();
			
			
		}
		std::chrono::time_point<std::chrono::system_clock> lEndTs = std::chrono::system_clock::now();
		std::cout<<"read @"<<(lCptr/std::chrono::duration<float>(lEndTs - lStartTs).count())<<"fps"<<std::endl;
		if(lCptr)
			std::cout<<"avg lat "<<lLatencyAvg / (float)lCptr<<" fps "<<lFpsAvg / (float)lCptr<<std::endl;
	}
	
	return 0;
}


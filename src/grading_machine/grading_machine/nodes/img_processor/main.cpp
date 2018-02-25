
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
#include "extract_thread.h"

class GradingMachineCppSettings : public settings_store::SettingsBase
{
public:
	GradingMachineCppSettings(ros::NodeHandle & pNodeHandle)
		: settings_store::SettingsBase(pNodeHandle)
		, mDebugImgChannels("yuvm")
		, mParamChanged(true)
		, mGaussianChannels("uv")
		, mDilateChannels("yuv")
	{
		registerAttribute<int>("grading_machine/filter_th_Y",mFilterParameters.mYParam.mThreshold,-1,255);
		registerAttribute<int>("grading_machine/filter_th_U",mFilterParameters.mUParam.mThreshold,-1,255);
		registerAttribute<int>("grading_machine/filter_th_V",mFilterParameters.mVParam.mThreshold,-1,255);
		registerAttribute<bool>("grading_machine/filter_erode",mFilterParameters.mErode);
		registerAttribute<float>("grading_machine/exclusion_percent_top",mFilterParameters.mExclusionZoneTopPercent,0.,0.25);
		registerAttribute<float>("grading_machine/exclusion_percent_bottom",mFilterParameters.mExclusionZoneBottomPercent,0.,0.25);
		
		registerAttribute<bool>("grading_machine/extract_connectivity_full",mExtractParameters.mConnectivityFullWay);
		
		
		
		registerAttribute<std::string>("grading_machine/filter_gaussian_channels",mGaussianChannels);
		registerAttribute<std::string>("grading_machine/filter_dilate_channels",mDilateChannels);
		
		registerAttribute<int32_t>("grading_machine/extract_min_pixel_per_area",mExtractParameters.mMinimumPixelsPerGroup,0,1024);
		registerAttribute<float>("grading_machine/extract_min_background_percent",mExtractParameters.mMinimumBackgroundPercent,0.,1.);
		
		registerAttribute<std::string>("grading_machine/debug_img_channels",mDebugImgChannels);
		
		declareAndRetrieveSettings();
	}
	
	virtual ~GradingMachineCppSettings()
	{
	}
	
	virtual void onParameterChanged(const std::string & pSettingName)
	{
		mParamChanged = true;
	}
	
	void updatePrameters()
	{
		if(!mParamChanged)
			return;
		
		mFilterParameters.mYParam.mGaussian = mGaussianChannels.find("y") != std::string::npos;
		mFilterParameters.mUParam.mGaussian = mGaussianChannels.find("u") != std::string::npos;
		mFilterParameters.mVParam.mGaussian = mGaussianChannels.find("v") != std::string::npos;
		
		mFilterParameters.mYParam.mDilate = mDilateChannels.find("y") != std::string::npos;
		mFilterParameters.mUParam.mDilate = mDilateChannels.find("u") != std::string::npos;
		mFilterParameters.mVParam.mDilate = mDilateChannels.find("v") != std::string::npos;
		
		mParamChanged = false;
	}
	
	FilterThread::Parameters	mFilterParameters;
	ExtractThread::Parameters	mExtractParameters;
	std::string 				mGaussianChannels;
	std::string 				mDilateChannels;
	std::string					mDebugImgChannels;
	bool						mParamChanged;
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
		image_transport::Publisher lPubA = it.advertise("A_image", 1);
		ros::Publisher lPubStat = n.advertise<grading_machine::ImgProcessorStat>("grading_machine/ImgProcessorStat",10);
		
		cv::Mat lYDownsized, lTmp, lRGB, lYUV;
		
		ros::Rate lLoopRate(1);
		
		GradingMachineCppSettings lSettings(n);
		
		
		
		CameraThread lCameraThread(CameraThread::Parameters(1600,1680,25));
		FilterThread lFilterThread(lCameraThread,lSettings.mFilterParameters);
		ExtractThread lExtractThread(lFilterThread,lSettings.mExtractParameters);
		lCameraThread.waitForCapture();

		GrabbedFrame lFrame;
		
		GrabbedFrame::tTimestamp lPreviousFrameTs = std::chrono::system_clock::now();
		GrabbedFrame::tTimestamp lStatStart;
		grading_machine::ImgProcessorStat lNextStat;
		int lStatCptr = 0;
		
		while(ros::ok())
		{
			lSettings.updatePrameters();
			if(lExtractThread.getNextFrame(lFrame))
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
				lNextStat.latency += std::chrono::duration<float,std::milli>(lNow - lFrame[GrabbedFrame::F_GrabDone]).count();
				lNextStat.filter += std::chrono::duration<float,std::milli>(lFrame[GrabbedFrame::F_FilterDone] - lFrame[GrabbedFrame::F_FilterStart]).count();
				lNextStat.areaextraction += std::chrono::duration<float,std::milli>(lFrame[GrabbedFrame::F_AreaExtractionDone] - lFrame[GrabbedFrame::F_AreaExtractionStart]).count();
				
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
				if( lSettings.mDebugImgChannels.find("a") != std::string::npos)
				{
					// downsize y
					cv::resize(lFrame[GrabbedFrame::Y],lYDownsized,cv::Size(lFrame[GrabbedFrame::BackgroundMask].size[1],lFrame[GrabbedFrame::BackgroundMask].size[0]));
					
					cv::bitwise_and(lYDownsized,lFrame[GrabbedFrame::BackgroundMask],lTmp);
					cv::swap(lYDownsized,lTmp);
					tAreas::iterator lIt = lFrame.editAreas().begin();
					const tAreas::iterator lItEnd = lFrame.editAreas().end();
					for( ; lIt != lItEnd ; ++lIt)
					{
						AreaOfInterest & lArea = *lIt;
						// cv::rectangle(lYDownsized,lArea.mAABBMin,lArea.mAABBMax,255);
						
						// rotated rectangle
						cv::Point2f rect_points[4]; 
						lArea.mOBB.points( rect_points );
						for( int j = 0; j < 4; j++ )
							cv::line( lYDownsized, rect_points[j], rect_points[(j+1)%4], 220, 1, 8 );
					}
					sensor_msgs::ImagePtr lMsg = cv_bridge::CvImage(std_msgs::Header(), "mono8", lYDownsized).toImageMsg();
					lPubA.publish(lMsg);
					// std::vector<cv::Mat> ch;
					// ch.push_back(lYDownsized);
					// ch.push_back(lFrame[GrabbedFrame::V]);
					// ch.push_back(lFrame[GrabbedFrame::U]);
					// cv::merge(ch,lYUV);
					
					// cv::cvtColor(lYUV,lRGB,cv::COLOR_YCrCb2RGB);
					
		
					// sensor_msgs::ImagePtr lMsg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", lRGB).toImageMsg();
					// lPubA.publish(lMsg);
				}
				
			}
			
			ros::spinOnce();

		}
		
	}
	
	return 0;
}



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

#include "grading_machine/ImgProcessorStat.h"
#include "image_common/camera_frame_provider.h"
#include "image_common/frame_debugger.h"
#include "img/filter_worker.h"
#include "img/extract_worker.h"
#include "areatracker.h"
#include "detectionmanager.h"

class GradingMachineCppSettings : public settings_store::SettingsBase
{
public:
	GradingMachineCppSettings(ros::NodeHandle & pNodeHandle)
		: settings_store::SettingsBase(pNodeHandle)
		, mDebugImgChannels("yuvm")
		, mGaussianChannels("uv")
		, mDilateChannels("yuv")
	{
		registerAttribute<int>("grading_machine/filter_th_Y",mFilterParameters.mYParam.mThreshold,-1,255,"threshold on Y -1=>automatic");
		registerAttribute<int>("grading_machine/filter_th_U",mFilterParameters.mUParam.mThreshold,-1,255,"threshold on U -1=>automatic");
		registerAttribute<int>("grading_machine/filter_th_V",mFilterParameters.mVParam.mThreshold,-1,255,"threshold on V -1=>automatic");
		registerAttribute<bool>("grading_machine/filter_erode",mFilterParameters.mErode,"should we erode combined mask ?");
		registerAttribute<float>("grading_machine/exclusion_percent_top",mFilterParameters.mExclusionZoneTopPercent,0.,0.25,"percent (of height) of pixels ignore in top");
		registerAttribute<float>("grading_machine/exclusion_percent_bottom",mFilterParameters.mExclusionZoneBottomPercent,0.,0.25,"percent (of height) of pixels ignore in bottom");

		registerAttribute<std::string>("grading_machine/filter_gaussian_channels",mGaussianChannels,"should we use a gaussian filter to remove noise ?");
		registerAttribute<std::string>("grading_machine/filter_dilate_channels",mDilateChannels,"should we dilate mask of each channel");
		
		registerAttribute<int32_t>("grading_machine/extract_min_pixel_per_area",mExtractParameters.mMinimumPixelsPerGroup,0,1024,"minimum pixel count in an area");
		registerAttribute<float>("grading_machine/extract_min_background_percent",mExtractParameters.mMinimumBackgroundPercent,0.,1.,"minimum percentage of frame that should be background");
		registerAttribute<float>("grading_machine/extract_min_area_dist_percent",mExtractParameters.mMinimumSpaceBetweenAreaPercent,0.,1.,"mimimum space in percent of width between two areas");
		registerAttribute<bool>("grading_machine/extract_connectivity_full",mExtractParameters.mConnectivityFullWay,"connexity used to extract ROI f=4,t=8");
		registerAttribute<int>("grading_machine/extract_max_area_count",mExtractParameters.mMaxAreaToExtract,1,25,"maximum number of expected areas");
		
		registerAttribute<std::string>("grading_machine/debug_img_channels",mDebugImgChannels,"which channels should be published for debug ? AYUVM");
		
		declareAndRetrieveSettings();
	}
	
	virtual ~GradingMachineCppSettings()
	{
	}
	
	virtual void onParameterChanged(const std::string & pSettingName)
	{
		if(pSettingName == "grading_machine/filter_gaussian_channels")
		{
			mFilterParameters.mYParam.mGaussian = mGaussianChannels.find("y") != std::string::npos;
			mFilterParameters.mUParam.mGaussian = mGaussianChannels.find("u") != std::string::npos;
			mFilterParameters.mVParam.mGaussian = mGaussianChannels.find("v") != std::string::npos;
		}
		else if(pSettingName == "grading_machine/filter_dilate_channels")
		{
			mFilterParameters.mYParam.mDilate = mDilateChannels.find("y") != std::string::npos;
			mFilterParameters.mUParam.mDilate = mDilateChannels.find("u") != std::string::npos;
			mFilterParameters.mVParam.mDilate = mDilateChannels.find("v") != std::string::npos;
		}
	}
	
	FilterWorker::Parameters	mFilterParameters;
	ExtractWorker::Parameters	mExtractParameters;
	std::string 				mGaussianChannels;
	std::string 				mDilateChannels;
	std::string					mDebugImgChannels;
};


int main(int argc, char ** argv)
{
	ros::init(argc,argv,"grading_machine_cpp");
	{
		cv::setNumThreads(1);
		
		ros::NodeHandle n;
		
		
		ros::Publisher lPubStat = n.advertise<grading_machine::ImgProcessorStat>("grading_machine/ImgProcessorStat",10);
		
		cv::Mat lTmp, lRGB, lYUV;
		
		ros::Rate lLoopRate(1);
		
		GradingMachineCppSettings lSettings(n);
		FrameDebugger lFrameDebugger(n,lSettings.mDebugImgChannels);
		
		
		
		CameraFrameProvider lCameraFrameProvider(CameraFrameProvider::Parameters(640,480,12));
		FilterWorker lFilterWorker(lCameraFrameProvider,lSettings.mFilterParameters);
		ExtractWorker lExtractWorker(lFilterWorker,lSettings.mExtractParameters);
		FrameProcessor<Frame> lExtractThread(lExtractWorker);
		
		Frame lFrame;
		
		DetectionManager lDetectionManager;
		AreaTracker lAreaTracker;
		lAreaTracker.setTrackingFunctor(&lDetectionManager);
		
		Frame::tTimestamp lPreviousFrameTs = std::chrono::system_clock::now();
		Frame::tTimestamp lStatStart;
		grading_machine::ImgProcessorStat lNextStat;
		int lStatCptr = 0;
		
		while(ros::ok())
		{
			if(lExtractThread.getNextFrame(lFrame))
			{
				// update stats
				{
					if(lStatCptr == 0)
					{
						lStatStart = std::chrono::system_clock::now();
						lNextStat = grading_machine::ImgProcessorStat();
					}
					++lStatCptr;
					
					Frame::tTimestamp lNow = std::chrono::system_clock::now();
					lNextStat.fps += 1.f/std::chrono::duration<float>(lNow - lPreviousFrameTs).count();
					lPreviousFrameTs = lNow;
					lNextStat.latency += std::chrono::duration<float,std::milli>(lNow - lFrame[Frame::F_GrabDone]).count();
					lNextStat.filter += std::chrono::duration<float,std::milli>(lFrame[Frame::F_FilterDone] - lFrame[Frame::F_FilterStart]).count();
					lNextStat.areaextraction += std::chrono::duration<float,std::milli>(lFrame[Frame::F_AreaExtractionDone] - lFrame[Frame::F_AreaExtractionStart]).count();
					
					//std::cout<<std::chrono::duration<float,std::milli>(lFrame[Frame::F_FilterDone] - lFrame[Frame::F_FilterStart]).count()<<" "<<std::chrono::duration<float,std::milli>(lFrame[Frame::F_AreaExtractionDone] - lFrame[Frame::F_AreaExtractionStart]).count()<<std::endl;
					
					if(std::chrono::duration<float>(lNow - lStatStart).count() > 0.25)
					{
						lNextStat.fps /= lStatCptr;
						lNextStat.latency /= lStatCptr;
						lNextStat.filter /= lStatCptr;
						lNextStat.areaextraction /= lStatCptr;
						lStatCptr = 0;
						lPubStat.publish(lNextStat);
						
					}
				}
				
				lAreaTracker.addNewFrame(lFrame);
				
				lFrameDebugger.setImage('y',lFrame[Frame::Y]);
				lFrameDebugger.setImage('u',lFrame[Frame::U]);
				lFrameDebugger.setImage('v',lFrame[Frame::V]);
				lFrameDebugger.setImage('m',lFrame[Frame::BackgroundMask]);
				if( lSettings.mDebugImgChannels.find('a') != std::string::npos)
				{
					
					cv::bitwise_and(lFrame[Frame::Y],lFrame[Frame::BackgroundMask],lTmp);
					lDetectionManager.computeDebugFrame(lTmp,lFrame);
					lFrameDebugger.setImage('a',lTmp);
				}
				cv::Mat lMat(128,128, CV_8UC3, cv::Scalar(0,0,255));
				lFrameDebugger.setImage('a',lMat);

			}
			
			ros::spinOnce();

		}
		
	}
	
	return 0;
}


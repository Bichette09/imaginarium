
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

class GradingMachineCppSettings : public settings_store::SettingsBase
{
public:
	GradingMachineCppSettings(ros::NodeHandle & pNodeHandle)
		: settings_store::SettingsBase(pNodeHandle)
		, mUpdateIntervalSec(1.)
	{
		registerAttribute<float>("grading_machine/update_period",mUpdateIntervalSec,0.5,10);
		
		declareAndRetrieveSettings();
	}
	
	virtual ~GradingMachineCppSettings()
	{
	}
	
	float mUpdateIntervalSec;
};

class MaskBuilder
{
public:
	MaskBuilder(cv::Mat & pTmpA, cv::Mat & pTmpB, const CameraThread::Parameters & pParameters)
		: mTmpA(pTmpA)
		, mTmpB(pTmpB)
	{
		mMorphoKernel = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(5,5));
		mTopEdgeRectA = cv::Point(0,0);
		mTopEdgeRectB = cv::Point(pParameters.mHalfWidth,(int)(pParameters.mHalfHeight * 0.06));
		mBottomEdgeRectA = cv::Point(0,pParameters.mHalfHeight - 1 - (int)(pParameters.mHalfHeight * 0.025));
		mBottomEdgeRectB = cv::Point(pParameters.mHalfWidth,pParameters.mHalfHeight - 1);
	}
	
	void computeMask(const cv::Mat & pU, const cv::Mat & pV, cv::Mat & pOut)
	{
		cv::threshold(pU,mTmpA,140,255,cv::THRESH_BINARY_INV + cv::THRESH_OTSU);
		cv::threshold(pV,mTmpB,150,255,cv::THRESH_BINARY_INV + cv::THRESH_OTSU );
		cv::bitwise_and(mTmpA,mTmpB,pOut);
		cv::bitwise_not(pOut,mTmpA);
		cv::swap(pOut,mTmpA);
		
		// clear top and bottom (we capture border of camera support)
		cv::rectangle(pOut,mTopEdgeRectA,mTopEdgeRectB,0,CV_FILLED);
		cv::rectangle(pOut,mBottomEdgeRectA,mBottomEdgeRectB,0,CV_FILLED);
		
	}
	
		
	void filterMask(cv::Mat & pInOut)
	{
		cv::morphologyEx(pInOut,mTmpA,cv::MORPH_DILATE,mMorphoKernel);
		cv::swap(pInOut,mTmpA);
		// cv::morphologyEx(mTmpA,pInOut,cv::MORPH_ERODE,mMorphoKernel);
		//cv::morphologyEx(mTmpA,pInOut,cv::MORPH_DILATE,mMorphoKernel);
	}
	
protected:
	cv::Mat 	mMorphoKernel;
	cv::Point 	mTopEdgeRectA;
	cv::Point 	mTopEdgeRectB;
	cv::Point 	mBottomEdgeRectA;
	cv::Point 	mBottomEdgeRectB;
	cv::Mat & 	mTmpA;
	cv::Mat & 	mTmpB;
};



int main(int argc, char ** argv)
{
	ros::init(argc,argv,"grading_machine_cpp");
	{
		
		ros::NodeHandle n;
		image_transport::ImageTransport it(n);
		image_transport::Publisher pubY = it.advertise("Y_image", 1);
		image_transport::Publisher pubU = it.advertise("U_image", 1);
		image_transport::Publisher pubV = it.advertise("V_image", 1);
		
		ros::Rate lLoopRate(4);
		
		GradingMachineCppSettings lSettings(n);
		
		
		
		CameraThread lCameraThread(CameraThread::Parameters(1600,1680,25));
		lCameraThread.waitForCapture();
		cv::Mat lY, lU, lV, lTmpMaskA, lTmpMaskB,lMask;
		cv::Mat lLabels;
		std::chrono::time_point<std::chrono::system_clock> lStartTs = std::chrono::system_clock::now();
		
		int lCptr = 0;
		
		MaskBuilder lMaskBuilder(lTmpMaskA,lTmpMaskB,lCameraThread.mParameters);
		
		
		while(ros::ok())
		{
			if(lCameraThread.hasNewFrame())
			{
				lCameraThread.getFrame(lY,lU,lV);
				++lCptr;
				
				lMaskBuilder.computeMask(lU,lV,lMask);
				
				
				lMaskBuilder.filterMask(lMask);
				
				int lLabelsCount = cv::connectedComponents( lMask,lLabels,4,CV_16U);
				// std::cout<<lLabelsCount<<"\n";
				// sensor_msgs::ImagePtr msgY = cv_bridge::CvImage(std_msgs::Header(), "mono8", lY).toImageMsg();
				// pubY.publish(msgY);
				
				lLabels *= (255 / lLabelsCount);
				
				
				sensor_msgs::ImagePtr msgU = cv_bridge::CvImage(std_msgs::Header(), "mono8", lMask).toImageMsg();
				pubU.publish(msgU);
				
				sensor_msgs::ImagePtr msgV = cv_bridge::CvImage(std_msgs::Header(), "mono16", lLabels).toImageMsg();
				pubV.publish(msgV);
				
				sensor_msgs::ImagePtr msgY = cv_bridge::CvImage(std_msgs::Header(), "mono8", lY).toImageMsg();
				pubY.publish(msgY);
			
			}
			// if(lCptr > 50)
				// break;
			
			// Camera.grab();
			// Camera.retrieve ( data);//get camera image
			// memcpy(lY.data,data,Camera.getHeight()*Camera.getWidth());
			// memcpy(lUsmall.data,data + Camera.getHeight()*Camera.getWidth(),(Camera.getHeight()*Camera.getWidth())/4);
			// memcpy(lVsmall.data,data + Camera.getHeight()*Camera.getWidth() + (Camera.getHeight()*Camera.getWidth())/4,(Camera.getHeight()*Camera.getWidth())/4);
			
			// cv::resize(lUsmall,lU,cv::Size(Camera.getWidth(),Camera.getHeight()),0.,0.,cv::INTER_NEAREST );
			// cv::resize(lVsmall,lV,cv::Size(Camera.getWidth(),Camera.getHeight()),0.,0.,cv::INTER_NEAREST );
			// std::vector<cv::Mat> channels;
			// channels.push_back(lGray);
			// channels.push_back(lU);
			// channels.push_back(lV);
			//cap.retrieve(frame);
			
			// sensor_msgs::ImagePtr msgU = cv_bridge::CvImage(std_msgs::Header(), "mono8", lUsmall).toImageMsg();
			// pubU.publish(msgU);
			// sensor_msgs::ImagePtr msgV = cv_bridge::CvImage(std_msgs::Header(), "mono8", lVsmall).toImageMsg();
			// pubV.publish(msgV);
			ros::spinOnce();
			
			
		}
		std::chrono::time_point<std::chrono::system_clock> lEndTs = std::chrono::system_clock::now();
		std::cout<<"read @"<<(lCptr/std::chrono::duration<float>(lEndTs - lStartTs).count())<<"fps"<<std::endl;
		
		// while(ros::ok())
		// {
			// ros::spinOnce();
			
			
			
			// std::chrono::time_point<std::chrono::system_clock> lNewTs = std::chrono::system_clock::now();
			// if (std::chrono::duration<float>(lNewTs - lTs).count() >= lSettings.mUpdateIntervalSec)
			// {
				// lTs = lNewTs;
				// ++lMsg.header.seq;
				// lMsg.header.stamp = ros::Time::now();
				// lMsg.cpuload = lHwMonitor.getCpuLoad();
				// lMsg.cputemp = lHwMonitor.getCpuTemperature();
				// lMsg.memload = lHwMonitor.getMemLoad();
				// lMsg.wifi = lHwMonitor.getNetworkLoad(HwMonitor::NI_Wifi);
				// lMsg.eth = lHwMonitor.getNetworkLoad(HwMonitor::NI_Ethernet);
				// lMsg.lo = lHwMonitor.getNetworkLoad(HwMonitor::NI_Loopback);
				// lMsg.wifistrength = lHwMonitor.getWifiStrenght();
				// lMsgPublisher.publish(lMsg);
				
				// std::cout<<"####\n";
				// lMsgPrinter.stream(std::cout," ",lMsg);

			// }
			// lLoopRate.sleep();
			
		// }
	}
	
	return 0;
}


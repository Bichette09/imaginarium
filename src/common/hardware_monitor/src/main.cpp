
// std
#include <iostream>
#include <fstream>
#include <chrono>
#include <stdio.h>

// ros
#include "ros/ros.h"
#include "std_msgs/String.h"

// settings_store
#include <settings_store/settings_store_client.h>
#include <settings_store/Change.h>

// hardware_monitor
#include <hardware_monitor/HardwareInfo.h>

// opencv
#include <opencv2/opencv.hpp>
#include <opencv2/core/ocl.hpp>

std::string exec(const char* cmd) 
{
	std::array<char, 128> buffer;
	std::string result;
	std::shared_ptr<FILE> pipe(popen(cmd, "r"), pclose);
	if (!pipe)
		return std::string();
	while (!feof(pipe.get())) {
		if (fgets(buffer.data(), 128, pipe.get()) != nullptr)
			result += buffer.data();
	}
	return result;
}

struct HwMonitor
{
	
	
	HwMonitor()
		: mPreviousThrottledFlags(0)
		, mVcGenIsMissing(false)
	{
		readCpuTicks(mPreviousLoad,mPreviousIdle);
		for(int i = 0 ; i < NI_Count ; ++i)
		{
			readNetworkInterfaceBytes(i,mPreviousNetworkReceivedBytes[i],mPreviousNetworkSendBytes[i]);
			mPreviousNetworkTs[i] = std::chrono::system_clock::now();
		}
		
		std::string lTest = exec("vcgencmd commans");
		if(lTest.empty())
		{
			mVcGenIsMissing = true;
			ROS_ERROR_STREAM( "vcgencmd is missing");
		}
		
	}
	
	enum NetworkInterface
	{
		NI_Loopback = 0,
		NI_Ethernet,
		NI_Wifi,
		
		NI_Count
	};
	
	float getCpuTemperature()
	{
		std::ifstream lStream;
		lStream.open("/sys/class/thermal/thermal_zone0/temp");
		int lTempMilliDegree = 0;
		lStream>>lTempMilliDegree;
		lStream.close();
		return std::floor(lTempMilliDegree / 100.f) / 10.f;
	}
	
	float getCpuLoad()
	{
		int64_t lNewLoad = 0;
		int64_t lNewIdle = 0;
		readCpuTicks(lNewLoad,lNewIdle);
		
		int64_t lDeltaLoad = lNewLoad - mPreviousLoad;
		int64_t lDeltaIdle = lNewIdle - mPreviousIdle;
		mPreviousLoad = lNewLoad;
		mPreviousIdle = lNewIdle;
		if( (lDeltaLoad + lDeltaIdle) <= 0)
			return -1.f;
		return std::floor(lDeltaLoad * 1000.f / (float)(lDeltaLoad + lDeltaIdle))/10.f; 
	}
	
	int getCpuFreq()
	{
		if(mVcGenIsMissing)
		{
			return 0;
		}
		
		std::string lFreqStr = exec("vcgencmd measure_clock arm");
		if(lFreqStr.size() <= 14)
		{
			mVcGenIsMissing = true;
			return 0;
		}
		// remove 'frequency(45)='
		lFreqStr = lFreqStr.substr(14);
		return strtol(lFreqStr.c_str(),NULL,10) / 1000000;
	}
	
	// https://www.raspberrypi.org/forums/viewtopic.php?f=63&t=147781&start=50#p972790
	// 0: under-voltage
	// 1: arm frequency capped
	// 2: currently throttled 
	// 16: under-voltage has occurred
	// 17: arm frequency capped has occurred
	// 18: throttling has occurred
	void checkThrottledFlags(settings_store::StateDeclarator & pStateDeclarator)
	{
		if(mVcGenIsMissing)
		{
			return;
		}
		
		std::string lFlags = exec("vcgencmd get_throttled");
		if(lFlags.size() <= 12)
		{
			mVcGenIsMissing = true;
			return;
		}
		
		// remove 'throttled=0x'
		int lNewFlags = (int)strtol( lFlags.substr(12).c_str(),NULL,16);
		// keep only first four bits
		lNewFlags &= 0xF;
		// print only states that were raised
		int lToPrint = lNewFlags & (~mPreviousThrottledFlags);
		mPreviousThrottledFlags = lNewFlags;
		
		if(lToPrint)
		{
			if(lToPrint & 0x01)
			{
				pStateDeclarator.setStateStr("Undervoltage","error");
			}
			if(lToPrint & 0x02)
			{
				pStateDeclarator.setStateStr("ArmFreqCapped","error");
			}
			if(lToPrint & 0x04)
			{
				pStateDeclarator.setStateStr("Throttled","error");
			}
			
			ROS_ERROR_STREAM( 
				(lToPrint & 0x01 ? "Undervoltage detected " : "") 
				<< (lToPrint & 0x02 ? "ArmFreqCapped detected " : "") 
				<< (lToPrint & 0x04 ? "Throttled detected " : "") 
				);
		}
	}
	
	float getMemLoad()
	{
		std::string lDummy,lId;
		int64_t lMemTotalKB = 0;
		int64_t lMemFreeKB = 0;
		std::ifstream lStream;
		lStream.open("/proc/meminfo");
		for(int i = 0 ; i < 3 ; ++i)
		{
			int64_t lValue = 0;
			lStream>>lId>>lValue>>lDummy;
			if(strcmp(lId.data(),"MemTotal:") == 0)
			{
				lMemTotalKB = lValue;
			}
			else if(strcmp(lId.data(),"MemFree:") == 0 && lMemFreeKB == 0)
			{
				lMemFreeKB = lValue;
			}
			else if(strcmp(lId.data(),"MemAvailable:") == 0 )
			{
				lMemFreeKB = lValue;
			}
		}
		lStream.close();
		if( lMemTotalKB <= 0)
			return -1.f;
		return std::floor( (lMemTotalKB - lMemFreeKB) * 1000.f / (float)lMemTotalKB)/10.f; 
	}
	
	float getNetworkLoad(NetworkInterface pInterface)
	{
		int64_t lNewR = 0;
		int64_t lNewS = 0;
		readNetworkInterfaceBytes(pInterface,lNewR,lNewS);
		std::chrono::time_point<std::chrono::system_clock> lNewTs = std::chrono::system_clock::now();
		
		int64_t lDeltaR = lNewR - mPreviousNetworkReceivedBytes[pInterface];
		int64_t lDeltaS = lNewS - mPreviousNetworkSendBytes[pInterface];
		float lDeltaTs = std::chrono::duration<float>(lNewTs - mPreviousNetworkTs[pInterface]).count();
		
		mPreviousNetworkReceivedBytes[pInterface] = lNewR;
		mPreviousNetworkSendBytes[pInterface] = lNewS;
		mPreviousNetworkTs[pInterface] = lNewTs;
		
		if(lDeltaTs <= 0.)
			return -1.f;
		float lDeltaKBytes = (lDeltaR + lDeltaS)/1024.;
			
		
		return std::floor(lDeltaKBytes * 10. / lDeltaTs) / 10.;
	}
	
	float getWifiStrenght()
	{
		std::ifstream lStream;
		lStream.open("/proc/net/wireless");
		// read two lines
		lStream.ignore(1024,'\n');
		lStream.ignore(1024,'\n');
		std::string lDummy;
		float lStrength = 0.;
		lStream>>lDummy>>lDummy>>lStrength;
		return lStrength;
	}
	
	static const std::string sNetworkInterfaceName[NI_Count];
	
private:
	
	void readCpuTicks(int64_t & pLoad, int64_t & pIdle) const
	{
		int64_t lUsrLoad = 0;
		int64_t lNiceLoad = 0;
		int64_t lSystemLoad = 0;
		int64_t lIdleLoad = 0;
		std::ifstream lStream;
		lStream.open("/proc/stat");
		std::string lCpuString;
		lStream>>lCpuString>>lUsrLoad>>lNiceLoad>>lSystemLoad>>lIdleLoad;
		lStream.close();
		
		pLoad = lUsrLoad + lNiceLoad + lSystemLoad;
		pIdle = lIdleLoad;
	}
	
	void readNetworkInterfaceBytes(int pInterface, int64_t & pReceived, int64_t & pSend)
	{
		std::ifstream lStreamR;
		lStreamR.open("/sys/class/net/" + sNetworkInterfaceName[pInterface] + "/statistics/rx_bytes");
		lStreamR>>pReceived;
		lStreamR.close();
		
		std::ifstream lStreamS;
		lStreamS.open("/sys/class/net/" + sNetworkInterfaceName[pInterface] + "/statistics/tx_bytes");
		lStreamS>>pSend;
		lStreamS.close();
	}
	
	
	
	int64_t		mPreviousLoad;
	int64_t		mPreviousIdle;
	int64_t		mPreviousNetworkReceivedBytes[NI_Count];
	int64_t		mPreviousNetworkSendBytes[NI_Count];
	std::chrono::time_point<std::chrono::system_clock> mPreviousNetworkTs[NI_Count];
	char		mSPrintfBuffer[64];
	int			mPreviousThrottledFlags;
	bool		mVcGenIsMissing;
};

const std::string HwMonitor::sNetworkInterfaceName[HwMonitor::NI_Count] = {"lo","eth0","wlan0"};

class HardwareMonitorSettings : public settings_store::SettingsBase
{
public:
	HardwareMonitorSettings(ros::NodeHandle & pNodeHandle)
		: settings_store::SettingsBase(pNodeHandle)
		, mUpdateIntervalSec(1.)
	{
		registerAttribute<float>("hardware_monitor/update_period",mUpdateIntervalSec,0.1,10,"hardware state read interval");
		
		declareAndRetrieveSettings();
	}
	
	virtual ~HardwareMonitorSettings()
	{
	}
	
	float mUpdateIntervalSec;
};

void detectOpenCLSupport()
{
	
	std::stringstream lStream;
	{
		lStream<<"OpenCL : ";
		if (!cv::ocl::haveOpenCL())
		{
			lStream << "NO support" << std::endl;
			goto detectOpenCLSupportEnd;
		}
		cv::ocl::Context context;
		if (!context.create(cv::ocl::Device::TYPE_ALL))
		{
			lStream << "Fail to create context" << std::endl;
			goto detectOpenCLSupportEnd;
		}

		// In OpenCV 3.0.0 beta, only a single device is detected.
		lStream << "supported on "<<context.ndevices() << " devices." << std::endl;
		for (int i = 0; i < context.ndevices(); i++)
		{
			cv::ocl::Device device = context.device(i);
			lStream << "####################"<<std::endl;
			lStream << "\tname                 : " << device.name() << std::endl;
			lStream << "\tavailable            : " << device.available() << std::endl;
			lStream << "\timageSupport         : " << device.imageSupport() << std::endl;
			lStream << "\tOpenCL_C_Version     : " << device.OpenCL_C_Version() << std::endl;
			lStream << std::endl;
		}
	}
detectOpenCLSupportEnd:
	ROS_INFO_STREAM(lStream.str());
}

int main(int argc, char ** argv)
{
	ros::init(argc,argv,"hardware_monitor");
	{
		
		ros::NodeHandle n;
		
		settings_store::StateDeclarator lStateDeclarator(n);
		
		// do not bufferize those messages
		ros::Publisher lMsgPublisher = n.advertise<hardware_monitor::HardwareInfo>("hardware_monitor/HardwareInfo", 1);
		hardware_monitor::HardwareInfo lMsg;
		lMsg.header.seq = 0;
		ros::message_operations::Printer<hardware_monitor::HardwareInfo> lMsgPrinter;
		
		
		std::unique_ptr<ros::Rate> lLoopRatePtr;
		float lCurrentRatePeriodSec = -1.;
		
		//std::cout<<"Starting hardware_monitor..."<<std::endl;
		HwMonitor lHwMonitor;
		
		
		HardwareMonitorSettings lSettings(n);
		
		ROS_INFO_STREAM("hardware_monitor ready !");
		detectOpenCLSupport();
		while(ros::ok())
		{
			ros::spinOnce();
			
			lHwMonitor.checkThrottledFlags(lStateDeclarator);
			
			++lMsg.header.seq;
			lMsg.header.stamp = ros::Time::now();
			lMsg.cpuload = lHwMonitor.getCpuLoad();
			lMsg.cputemp = lHwMonitor.getCpuTemperature();
			lMsg.memload = lHwMonitor.getMemLoad();
			lMsg.wifi = lHwMonitor.getNetworkLoad(HwMonitor::NI_Wifi);
			lMsg.eth = lHwMonitor.getNetworkLoad(HwMonitor::NI_Ethernet);
			lMsg.lo = lHwMonitor.getNetworkLoad(HwMonitor::NI_Loopback);
			lMsg.wifistrength = lHwMonitor.getWifiStrenght();
			lMsg.cpufreq = lHwMonitor.getCpuFreq();
			lMsgPublisher.publish(lMsg);
			
			if(lCurrentRatePeriodSec != lSettings.mUpdateIntervalSec)
			{
				lLoopRatePtr.reset(new ros::Rate(1./lSettings.mUpdateIntervalSec));
				lCurrentRatePeriodSec = lSettings.mUpdateIntervalSec;
			}
			lLoopRatePtr->sleep();
			
		}
	}
	
	return 0;
}


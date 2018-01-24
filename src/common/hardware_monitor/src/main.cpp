
// std
#include <iostream>
#include <fstream>
#include <chrono>

// ros
#include "ros/ros.h"
#include "std_msgs/String.h"

// settings_store
#include <settings_store/settings_store_client.h>

// hardware_monitor
#include <hardware_monitor/msg.h>



struct HwMonitor
{
	HwMonitor()
	{
		readCpuTicks(mPreviousLoad,mPreviousIdle);
		for(int i = 0 ; i < NI_Count ; ++i)
		{
			readNetworkInterfaceBytes(i,mPreviousNetworkReceivedBytes[i],mPreviousNetworkSendBytes[i]);
			mPreviousNetworkTs[i] = std::chrono::system_clock::now();
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
};

const std::string HwMonitor::sNetworkInterfaceName[HwMonitor::NI_Count] = {"lo","eth0","wlan0"};

class HardwareMonitorSettings : public settings_store::SettingsBase
{
public:
	HardwareMonitorSettings(ros::NodeHandle & pNodeHandle)
		: settings_store::SettingsBase(pNodeHandle)
		, mUpdateIntervalSec(1.)
	{
		registerAttribute<float>("hardware_monitor/update_period",mUpdateIntervalSec,0.5,10);
		
		declareAndRetrieveSettings();
	}
	
	virtual ~HardwareMonitorSettings()
	{
	}
	
	float mUpdateIntervalSec;
};

int main(int argc, char ** argv)
{
	ros::init(argc,argv,"hardware_monitor");
	{
		
		ros::NodeHandle n;
		
		// do not bufferize those messages
		ros::Publisher lMsgPublisher = n.advertise<hardware_monitor::msg>("hardware_monitor", 1);
		hardware_monitor::msg lMsg;
		lMsg.header.seq = 0;
		ros::message_operations::Printer<hardware_monitor::msg> lMsgPrinter;
		
		ros::Rate lLoopRate(5);//1Hz
		
		//std::cout<<"Starting hardware_monitor..."<<std::endl;
		HwMonitor lHwMonitor;
		
		
		HardwareMonitorSettings lSettings(n);
		
		std::chrono::time_point<std::chrono::system_clock> lTs = std::chrono::system_clock::now();
		
		std::cout<<"hardware_monitor ready !"<<std::endl;
		while(ros::ok())
		{
			// do work every 2s
			std::chrono::time_point<std::chrono::system_clock> lNewTs = std::chrono::system_clock::now();
			if (std::chrono::duration<float>(lNewTs - lTs).count() >= 1.)
			{
				lTs = lNewTs;
				++lMsg.header.seq;
				lMsg.header.stamp = ros::Time::now();
				lMsg.cpuload = lHwMonitor.getCpuLoad();
				lMsg.cputemp = lHwMonitor.getCpuTemperature();
				lMsg.memload = lHwMonitor.getMemLoad();
				lMsg.wifi = lHwMonitor.getNetworkLoad(HwMonitor::NI_Wifi);
				lMsg.eth = lHwMonitor.getNetworkLoad(HwMonitor::NI_Ethernet);
				lMsg.lo = lHwMonitor.getNetworkLoad(HwMonitor::NI_Loopback);
				lMsg.wifistrength = lHwMonitor.getWifiStrenght();
				lMsgPublisher.publish(lMsg);
				
				//std::cout<<"####\n";
				//lMsgPrinter.stream(std::cout," ",lMsg);

			}
			lLoopRate.sleep();
			
		}
	}
	
	return 0;
}


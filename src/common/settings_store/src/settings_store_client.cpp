#include <settings_store_client.hpp>

// ros
#include "std_msgs/String.h"


#include <settings_store/declareandget.h>

using namespace settings_store;


SettingsBase::SettingsBase(ros::NodeHandle & pNodeHandle)
	: mNodeHandle(pNodeHandle)
{
	mChangeTopic = mNodeHandle.subscribe("settings_store/Change", 1000, &SettingsBase::onChange,this);
}

SettingsBase::~SettingsBase()
{
	
}


void SettingsBase::onChange(const settings_store::Change::ConstPtr& pMsg)
{

}


SettingsBase::SettingInfo::SettingInfo()
	: mType(T_Unknown)
	, mPtr(NULL)
	, mIsDeclared(false)
{
}
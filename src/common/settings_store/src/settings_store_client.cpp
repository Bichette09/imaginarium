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
	setValue(pMsg.name,pMsg.value);
}

void SettingsBase::setValue(const std::string & pName, const std::string & pValue)
{
	tSettings::iterator lFindIt = mSettings.find(pMsg.name);
	if(lFindIt == mSettings.end())
		return;
	SettingInfo & lInfo = lFindIt->second;
	if(!lInfo.mIsDeclared)
		return;
	lInfo.setValueFromString(pValue);
}

void SettingsBase::declareAndRetrieveValues()
{
	std::vector<std::string> lNames;
	std::vector<std::string> lValues;
	
	tSettings::iterator lIt = mSettings.begin();
	const tSettings::iterator lItEnd = mSettings.end();
	for( ; lIt != lItEnd ; ++lIt)
	{
		if(!lIt->second.mIsDeclared)
		{
			lNames.push_back(lIt->first);
			lValues.push_back(lIt->second.getValueAsString());
		}
	}
	
	
	
}

SettingsBase::SettingInfo::SettingInfo()
	: mType(T_Unknown)
	, mPtr(NULL)
	, mIsDeclared(false)
{
}

void SettingsBase::SettingInfo::setValueFromString(const std::string & pValue)
{
	switch(lInfo.mType)
	{
		case T_Bool:
		{
			bool & lDst = *reinterpret_cast<bool*>(lInfo.mPtr);
			std::string lTmp = pValue;
			std::transform(lTmp.begin(), lTmp.end(), lTmp.begin(), ::tolower);
			lDst = lTmp == "t" || lTmp == "true" || lTmp == "1";
			return;
		}
		case T_Int16:
		{
			int16_t & lDst = *reinterpret_cast<int16_t*>(lInfo.mPtr);
			lDst = static_cast<int16_t>(atoi(pValue.c_str()));
			return;
		}
		case T_UInt16:
		{
			uint16_t & lDst = *reinterpret_cast<uint16_t*>(lInfo.mPtr);
			lDst = static_cast<uint16_t>(atoi(pValue.c_str()));
			return;
		}
		case T_Int32:
		{
			int32_t & lDst = *reinterpret_cast<int32_t*>(lInfo.mPtr);
			lDst = static_cast<int32_t>(atoi(pValue.c_str()));
			return;
		}
		case T_UInt32:
		{
			uint32_t & lDst = *reinterpret_cast<uint32_t*>(lInfo.mPtr);
			lDst = static_cast<uint32_t>(atoi(pValue.c_str()));
			return;
		}
		case T_Int64:
		{
			int64_t & lDst = *reinterpret_cast<int64_t*>(lInfo.mPtr);
			lDst = static_cast<int16_t>(atoll(pValue.c_str()));
			return;
		}
		case T_UInt64:
		{
			uint64_t & lDst = *reinterpret_cast<uint64_t*>(lInfo.mPtr);
			lDst = static_cast<uint64_t>(atoll(pValue.c_str()));
			return;
		}
		case T_Float:
		{
			float & lDst = *reinterpret_cast<float*>(lInfo.mPtr);
			lDst = static_cast<float>(atof(pValue.c_str()));
			return;
		}
		case T_Double:
		{
			double & lDst = *reinterpret_cast<double*>(lInfo.mPtr);
			lDst = static_cast<double>(atod(pValue.c_str()));
			return;
		}
		case T_String:
		{
			std::string & lDst = *reinterpret_cast<std::string*>(lInfo.mPtr);
			lDst = pValue;
			return;
		}
		default:
		{
			ROS_ERROR_STREAM("Unhandled param type"<<lInfo.mType);
			break;
		}
	}
}

template <typename T>
std::string toString(void * pPtr)
{
	T & lVal = *reinterpret_cast<T*>(lInfo.mPtr);
	std::stringstream lStream;
	lStream<<lVal;
	return lStream.str();
}

std::string SettingsBase::SettingInfo::getValueAsString() const
{
	switch(lInfo.mType)
	{
		case T_Bool:
		{
			bool & lVal = *reinterpret_cast<bool*>(lInfo.mPtr);
			return lVal ? "true":"false";
		}
		case T_Int16:
		{
			return toString<int16_t>(lInfo.mPtr);
		}
		case T_UInt16:
		{
			return toString<uint16_t>(lInfo.mPtr);
		}
		case T_Int32:
		{
			return toString<int32_t>(lInfo.mPtr);
		}
		case T_UInt32:
		{
			return toString<uint32_t>(lInfo.mPtr);
		}
		case T_Int64:
		{
			return toString<int64_t>(lInfo.mPtr);
		}
		case T_UInt64:
		{
			return toString<uint64_t>(lInfo.mPtr);
		}
		case T_Float:
		{
			return toString<float>(lInfo.mPtr);
		}
		case T_Double:
		{
			return toString<double>(lInfo.mPtr);
		}
		case T_String:
		{
			std::string & lVal = *reinterpret_cast<std::string*>(lInfo.mPtr);
			return lVal;
		}
		default:
		{
			ROS_ERROR_STREAM("Unhandled param type"<<lInfo.mType);
			break;
		}
	}
}
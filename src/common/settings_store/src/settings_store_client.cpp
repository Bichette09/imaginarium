#include <settings_store/settings_store_client.h>

// ros
#include "std_msgs/String.h"


#include <settings_store/declareandget.h>

using namespace settings_store;


SettingsBase::SettingsBase(ros::NodeHandle & pNodeHandle)
	: mNodeHandle(pNodeHandle)
{
	ros::service::waitForService("/settings_store/declareandget",5000);
	mDeclareAndGetService = mNodeHandle.serviceClient<settings_store::declareandget>("/settings_store/declareandget");
	mChangeTopic = mNodeHandle.subscribe("/settings_store/Change", 1000, &SettingsBase::onChange, this);
}

SettingsBase::~SettingsBase()
{
}

void SettingsBase::onParameterChanged(const std::string & /*pSettingName*/)
{
}

void SettingsBase::onChange(const settings_store::Change::ConstPtr& pMsg)
{
	setValue(pMsg->name,pMsg->value);
}

void SettingsBase::setValue(const std::string & pName, const std::string & pValue)
{
	tSettings::iterator lFindIt = mSettings.find(pName);
	if(lFindIt == mSettings.end())
		return;
	SettingInfo & lInfo = lFindIt->second;
	if(!lInfo.mIsDeclared)
		return;
	lInfo.setValueFromString(pValue);
	onParameterChanged(pName);
}

void SettingsBase::declareAndRetrieveSettings()
{
	settings_store::declareandget lParam;
	
	tSettings::iterator lIt = mSettings.begin();
	const tSettings::iterator lItEnd = mSettings.end();
	for( ; lIt != lItEnd ; ++lIt)
	{
		if(!lIt->second.mIsDeclared)
		{
			lParam.request.names.push_back(lIt->first);
			lParam.request.defaultvalues.push_back(lIt->second.getValueAsString());
			lParam.request.descriptions.push_back(lIt->second.mDescription);
		}
	}
	
	if(lParam.request.names.empty())
		return;
	if(!mDeclareAndGetService.call(lParam))
	{
		std::stringstream lVals;
		for( tSettings::iterator lIt = mSettings.begin() ; lIt != mSettings.end() ; ++lIt)
		{
			if(lIt != mSettings.begin())
				lVals<<", ";
			lVals<<lIt->first;
		}
		ROS_ERROR_STREAM("fail to call declareandget service "<<lVals.c_str());
		return;
	}
	
	settings_store::declareandget::Request::_names_type::const_iterator lItName = lParam.request.names.begin();
	const settings_store::declareandget::Request::_names_type::const_iterator lItNameEnd = lParam.request.names.end();
	settings_store::declareandget::Response::_values_type::const_iterator lItValue = lParam.response.values.begin();
	const settings_store::declareandget::Response::_values_type::const_iterator lItValueEnd = lParam.response.values.end();
	for( ; lItName != lItNameEnd && lItValue != lItValueEnd; ++lItName,++lItValue)
	{
		tSettings::iterator lFindIt = mSettings.find(*lItName);
		if(lFindIt == mSettings.end())
			continue;
		lFindIt->second.mIsDeclared = true;
		setValue(*lItName,*lItValue);
	}
}

SettingsBase::SettingInfo::SettingInfo()
	: mType(T_Unknown)
	, mPtr(NULL)
	, mIsDeclared(false)
{
}

void SettingsBase::SettingInfo::setValueFromString(const std::string & pValue,bool pDisableRangeCheck)
{
	switch(mType)
	{
		case T_Bool:
		{
			bool & lDst = *reinterpret_cast<bool*>(mPtr);
			std::string lTmp = pValue;
			std::transform(lTmp.begin(), lTmp.end(), lTmp.begin(), ::tolower);
			lDst = lTmp == "t" || lTmp == "true" || lTmp == "1";
			return;
		}
		case T_Int16:
		{
			setNumericValue(static_cast<int16_t>(atoi(pValue.c_str())),pDisableRangeCheck);
			return;
		}
		case T_UInt16:
		{
			setNumericValue(static_cast<uint16_t>(atoi(pValue.c_str())),pDisableRangeCheck);
			return;
		}
		case T_Int32:
		{
			setNumericValue(static_cast<int32_t>(atoi(pValue.c_str())),pDisableRangeCheck);
			return;
		}
		case T_UInt32:
		{
			setNumericValue(static_cast<uint32_t>(atoi(pValue.c_str())),pDisableRangeCheck);
			return;
		}
		case T_Int64:
		{
			setNumericValue(static_cast<int64_t>(atoll(pValue.c_str())),pDisableRangeCheck);
			return;
		}
		case T_UInt64:
		{
			setNumericValue(static_cast<uint64_t>(atoll(pValue.c_str())),pDisableRangeCheck);
			return;
		}
		case T_Float:
		{
			setNumericValue(static_cast<float>(atof(pValue.c_str())),pDisableRangeCheck);
			return;
		}
		case T_Double:
		{
			setNumericValue(static_cast<double>(atof(pValue.c_str())),pDisableRangeCheck);
			return;
		}
		case T_String:
		{
			std::string & lDst = *reinterpret_cast<std::string*>(mPtr);
			lDst = pValue;
			return;
		}
		default:
		{
			ROS_ERROR_STREAM("Unhandled param type"<<mType);
			break;
		}
	}
}

template <typename T>
std::string toString(void * pPtr)
{
	T & lVal = *reinterpret_cast<T*>(pPtr);
	std::stringstream lStream;
	lStream<<lVal;
	return lStream.str();
}

std::string SettingsBase::SettingInfo::getValueAsString() const
{
	switch(mType)
	{
		case T_Bool:
		{
			bool & lVal = *reinterpret_cast<bool*>(mPtr);
			return lVal ? "true":"false";
		}
		case T_Int16:
		{
			return toString<int16_t>(mPtr);
		}
		case T_UInt16:
		{
			return toString<uint16_t>(mPtr);
		}
		case T_Int32:
		{
			return toString<int32_t>(mPtr);
		}
		case T_UInt32:
		{
			return toString<uint32_t>(mPtr);
		}
		case T_Int64:
		{
			return toString<int64_t>(mPtr);
		}
		case T_UInt64:
		{
			return toString<uint64_t>(mPtr);
		}
		case T_Float:
		{
			return toString<float>(mPtr);
		}
		case T_Double:
		{
			return toString<double>(mPtr);
		}
		case T_String:
		{
			std::string & lVal = *reinterpret_cast<std::string*>(mPtr);
			return lVal;
		}
		default:
		{
			ROS_ERROR_STREAM("Unhandled param type"<<mType);
			break;
		}
	}
}

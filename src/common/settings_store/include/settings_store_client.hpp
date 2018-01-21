#pragma once

// std
#include <map>
#include <string>

// ros
#include "ros/ros.h"

#include <settings_store/Change.h>

namespace settings_store
{
	class SettingsBase
	{
	public:
		SettingsBase(ros::NodeHandle & pNodeHandle);
		~SettingsBase();
		
	protected:
		/** call this method to add an attribute once all attribute were added call declareAndRetrieveValues
		*	@param pSettingName is the key name in settings_store
		*	@param pAttribute is the attribute that should be mapped to the setting
		*/
		template <typename T>
		void addSetting(const std::string & pSettingName, T & pAttribute);
		
		void declareAndRetrieveValues();
	private:
		void onChange(const settings_store::Change::ConstPtr& pMsg);
		
		enum Type
		{
			T_Unknown,
			T_Bool,
			T_Int16,
			T_UInt16,
			T_Int32,
			T_UInt32,
			T_Int64,
			T_UInt64,
			T_Float,
			T_Double,
			T_String
		};
		struct SettingInfo
		{
			SettingInfo();
			
			std::string		mDistantName;
			Type			mType;
			void *			mPtr;
			bool 			mIsDeclared;
		};
		
		typedef std::map<std::string,SettingInfo> tSettings;
		tSettings			mSettings;
		ros::NodeHandle & 	mNodeHandle;
		ros::Subscriber		mChangeTopic;
	};
}

#include <settings_store_client.inl>
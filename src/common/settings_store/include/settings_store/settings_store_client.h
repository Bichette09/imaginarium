#pragma once

// std
#include <map>
#include <string>
#include <limits>

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
		void registerAttribute(const std::string & pSettingName, T & pAttribute, const char * pDescription = NULL);
		template <typename T>
		void registerAttribute(const std::string & pSettingName, T & pAttribute,T pMin, T pMax, const char * pDescription = NULL);
		
		void declareAndRetrieveSettings();
		
		/** this method is called after update of the given parameter
		*/
		virtual void onParameterChanged(const std::string & pSettingName);
	private:
		void onChange(const settings_store::Change::ConstPtr& pMsg);
		void setValue(const std::string & pName, const std::string & pValue);
		
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
		
		template <typename T>
		static Type GetTypeOf();
		
		struct SettingInfo
		{
			SettingInfo();
			
			void setValueFromString(const std::string & pValue, bool pDisableRangeCheck = false);
			template <typename T>
			void setNumericValue(T pVal,bool pDisableRangeCheck);
			std::string getValueAsString() const;
			
			
			
			std::string		mDistantName;
			std::string		mDescription;
			Type			mType;
			void *			mPtr;
			char			mMin[8];
			char			mMax[8];
			std::string		mDefault;
			bool 			mIsDeclared;
		};
		
		typedef std::map<std::string,SettingInfo> tSettings;
		tSettings			mSettings;
		ros::NodeHandle & 	mNodeHandle;
		ros::Subscriber		mChangeTopic;
		ros::ServiceClient	mDeclareAndGetService;
	};
	
	class StateDeclarator
	{
	public:
		StateDeclarator(ros::NodeHandle & pNodeHandle);
		~StateDeclarator();
		
		template <typename T>
		void setState(const std::string & pName, const T & pValue);
		
		void setStateStr(const std::string & pName, const std::string & pValue);
		
	private:
		ros::NodeHandle & 	mNodeHandle;
		ros::Publisher		mPublisher;
	};
}

#include <settings_store/settings_store_client.inl>

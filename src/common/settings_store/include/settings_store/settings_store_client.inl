

namespace settings_store
{
	template <>
	SettingsBase::Type SettingsBase::GetTypeOf<bool>() { return T_Bool;}

	template <>
	SettingsBase::Type SettingsBase::GetTypeOf<int16_t>() { return T_Int16;}

	template <>
	SettingsBase::Type SettingsBase::GetTypeOf<uint16_t>() { return T_UInt16;}

	template <>
	SettingsBase::Type SettingsBase::GetTypeOf<int32_t>() { return T_Int32;}

	template <>
	SettingsBase::Type SettingsBase::GetTypeOf<uint32_t>() { return T_UInt32;}

	template <>
	SettingsBase::Type SettingsBase::GetTypeOf<int64_t>() { return T_Int64;}

	template <>
	SettingsBase::Type SettingsBase::GetTypeOf<uint64_t>() { return T_UInt64;}

	template <>
	SettingsBase::Type SettingsBase::GetTypeOf<float>() { return T_Float;}

	template <>
	SettingsBase::Type SettingsBase::GetTypeOf<double>() { return T_Double;}

	template <>
	SettingsBase::Type SettingsBase::GetTypeOf<std::string>() { return T_String; }

	
#define GET_SETTING_INFO \
	if(mSettings.find(pSettingName) != mSettings.end())\
	{\
		ROS_ERROR_STREAM("Attribute "<<pSettingName<<" is already registered");\
		return;\
	}\
	SettingInfo & lSettingInfo = mSettings[pSettingName];\
	lSettingInfo.mDistantName = pSettingName;\
	

	
	template <>
	void SettingsBase::registerAttribute<std::string>(const std::string & pSettingName, std::string & pAttribute, const char * pDescription)
	{
		GET_SETTING_INFO;
		lSettingInfo.mType = T_String;
		lSettingInfo.mPtr = &pAttribute;
		lSettingInfo.mDefault = lSettingInfo.getValueAsString();
		lSettingInfo.mDescription = pDescription ? std::string(pDescription) : std::string();
	}
	
	template <>
	void SettingsBase::registerAttribute<bool>(const std::string & pSettingName, bool & pAttribute, const char * pDescription)
	{
		GET_SETTING_INFO;
		lSettingInfo.mType = T_Bool;
		lSettingInfo.mPtr = &pAttribute;
		lSettingInfo.mDefault = lSettingInfo.getValueAsString();
		lSettingInfo.mDescription = pDescription ? std::string(pDescription) : std::string();
	}
	
	template <typename T>
	void SettingsBase::registerAttribute(const std::string & pSettingName, T & pAttribute, const char * pDescription)
	{
		registerAttribute(pSettingName,pAttribute,std::numeric_limits<T>::lowest(),std::numeric_limits<T>::max(),pDescription);
	}
	
	template <typename T>
	void SettingsBase::registerAttribute(const std::string & pSettingName, T & pAttribute, T pMin, T pMax, const char * pDescription)
	{
		GET_SETTING_INFO;
		lSettingInfo.mType = GetTypeOf<T>();
		lSettingInfo.mPtr = &pAttribute;
		lSettingInfo.mDefault = lSettingInfo.getValueAsString();
		lSettingInfo.mDescription = pDescription ? std::string(pDescription) : std::string();
		memcpy(&lSettingInfo.mMin,&pMin,sizeof(T));
		memcpy(&lSettingInfo.mMax,&pMax,sizeof(T));
	}

#undef GET_SETTING_INFO

	template <typename T>
	void SettingsBase::SettingInfo::setNumericValue(T pVal, bool pDisableRangeCheck)
	{
		T lMin, lMax;
		memcpy(&lMin,mMin,sizeof(T));
		memcpy(&lMax,mMax,sizeof(T));
		if((!pDisableRangeCheck) && (lMin > pVal || lMax < pVal))
		{
			ROS_WARN_STREAM(mDistantName<<" value "<<pVal<<" is out of range ["<<lMin<<";"<<lMax<<"], default to "<<mDefault);
			// disable range check to avoid infinite loop if default is not in range
			setValueFromString(mDefault,true);
		}
		else
		{
			memcpy(mPtr,&pVal,sizeof(T));
		}
	}
	
}

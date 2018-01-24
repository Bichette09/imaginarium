
/*template <typename T>
Type GetTypeOf() {ROS_FATAL_STREAM("GetType called with unknown type"); return T_Unknown; }
*/

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
	void SettingsBase::registerAttribute<std::string>(const std::string & pSettingName, std::string & pAttribute)
	{
		GET_SETTING_INFO;
		lSettingInfo.mType = T_String;
		lSettingInfo.mPtr = &pAttribute;
		lSettingInfo.mDefault = lSettingInfo.getValueAsString();
	}
	
	template <>
	void SettingsBase::registerAttribute<bool>(const std::string & pSettingName, bool & pAttribute)
	{
		GET_SETTING_INFO;
		lSettingInfo.mType = T_Bool;
		lSettingInfo.mPtr = &pAttribute;
		lSettingInfo.mDefault = lSettingInfo.getValueAsString();
	}
	
	template <typename T>
	void SettingsBase::registerAttribute(const std::string & pSettingName, T & pAttribute)
	{
		registerAttribute(pSettingName,pAttribute,std::numeric_limits<T>::lowest(),std::numeric_limits<T>::max());
	}
	
	template <typename T>
	void SettingsBase::registerAttribute(const std::string & pSettingName, T & pAttribute, T pMin, T pMax)
	{
		GET_SETTING_INFO;
		lSettingInfo.mType = GetTypeOf<T>();
		lSettingInfo.mPtr = &pAttribute;
		lSettingInfo.mDefault = lSettingInfo.getValueAsString();
		*reinterpret_cast<T*>(lSettingInfo.mMin) = pMin;
		*reinterpret_cast<T*>(lSettingInfo.mMax) = pMax;
	}

#undef GET_SETTING_INFO
}

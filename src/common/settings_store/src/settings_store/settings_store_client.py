#!/usr/bin/env python
# coding: utf-8

import rospy
import settings_store.msg
import settings_store.srv

## base class that should be derived to declare/synchronize settings with settings_store server
# 
class SettingsBase:

	def __init__(self):
		self.__mAttributes = {}
		rospy.Subscriber('settings_store/Change',settings_store.msg.change,self)
		rospy.wait_for_service('/settings_store/declareandget',2.)
		self.__mServiceDeclareAndGet = rospy.ServiceProxy('/settings_store/declareandget', settings_store.srv.declareandget)
	
	class SettingInfo:
		def __init__(self, pName = None, pDefault = None, pMin = None, pMax = None):
			self.mAttributeName = pName
			self.mMin = pMin
			self.mMax = pMax
			self.mDefault = pDefault
		
	## this method will register given attributes to sync them with settings store
	# pAttributes is a list of tuple (local_attribute_name, settings_store_key_name, numeric_min, numeric_max)
	# local_attribute_name : is the name of attribute in sub class eg mMyAttribute
	# settings_store_key_name : is the key name in settings_store eg MyPackage/MySetting
	# numeric_min / numeric_max : are used for numeric attributes to ensure that value retrieve from the store is acceptable, if not default value will be used
	def registerAttributes(self, pAttributes):
		lNames = []
		lDefaultValues = []
		
		for (local, distant, min, max) in pAttributes:
			if distant in self.__mAttributes:
				raise Exception('setting ' + distant + ' is already associated to attribute ' + self.__mAttributes[distant].mAttributeName)
			# ensure that pLocalName exists and retrieve default value
			lDefaultValues.append(str(getattr(self,local)));
			self.__mAttributes[distant] = SettingInfo(local,getattr(self,local), min, max)
			
			lNames.append(distant)
		lServiceReq = settings_store.srv.declareandgetRequest(lNames, lDefaultValues)
		lResponse = self.__mServiceDeclareAndGet(lServiceReq)
		
		for ((local,distant),val) in zip(pAttributes,lResponse.values):
			self.__setAttrValFromStr(distant,val)
	
	def __setNumericAttr(self, pSettingInfo, pVal):
		if pSettingInfo.mMin is None or (pSettingInfo.mMin <= lVal and lVal <= pSettingInfo.mMax):
			setattr(self,pName,lVal)
		else
			rospy.logwarn('Setting out of range % % % %',pSettingInfo.mAttributeName,pStrVal,pSettingInfo.mMin,pSettingInfo.mMax)
			setattr(self,pName,pSettingInfo.mDefault)
	
	def __setAttrValFromStr(self, pDistant, pStrVal):
		lSettingInfo = self.__mAttributes[pDistant]
		lCurrentValue = getattr(self,lSettingInfo.mAttributeName)
		
		if isinstance(lCurrentValue,str):
			setattr(self,pName,str(pStrVal))
		elif isinstance(lCurrentValue,int):
			self.__setNumericAttr(lSettingInfo,int(pStrVal))
		elif isinstance(lCurrentValue,long):
			self.__setNumericAttr(lSettingInfo,long(pStrVal))
		elif isinstance(lCurrentValue,float):
			self.__setNumericAttr(lSettingInfo,float(pStrVal))
		elif isinstance(lCurrentValue,bool):
			setattr(self,pName,bool(pStrVal))
		else:
			rospy.logfatal('type of ' + pName + ' is not handled')
	
	def __call__(self,pParam):
		if not pParam.name in self.__mAttributes:
			return
		lLocalName = self.__mAttributes[pParam.name]
		self.__setAttrValFromStr(lLocalName,pParam.value)
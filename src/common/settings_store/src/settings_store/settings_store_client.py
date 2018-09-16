#!/usr/bin/env python
# coding: utf-8

import rospy
import settings_store.msg
import settings_store.srv
import datetime
import std_msgs



## base class that should be derived to declare/synchronize settings with settings_store server
# 
class SettingsBase:

		
	def __init__(self):
		self.__mAttributes = {}
		rospy.Subscriber('settings_store/Change',settings_store.msg.Change,self.__onSettingChangeCallback)
		rospy.wait_for_service('/settings_store/declareandget',10.)
		self.__mServiceDeclareAndGet = rospy.ServiceProxy('/settings_store/declareandget', settings_store.srv.declareandget)
	
	class SettingInfo:
		def __init__(self, pName = None, pDefault = None, pMin = None, pMax = None, pDescription = None):
			self.mAttributeName = pName
			self.mMin = pMin
			self.mMax = pMax
			self.mDefault = pDefault
			self.mDescription = pDescription
		
	## this method will register given attributes to sync them with settings store
	# pAttributes is a list of tuple (local_attribute_name, settings_store_key_name, [numeric_min, numeric_max], [description])
	# local_attribute_name : is the name of attribute in sub class eg mMyAttribute
	# settings_store_key_name : is the key name in settings_store eg MyPackage/MySetting
	# numeric_min / numeric_max : are used for numeric attributes to ensure that value retrieve from the store is acceptable, if not default value will be used
	def registerAttributes(self, pAttributes):
		lNames = []
		lDefaultValues = []
		lDescriptions = []
		
		for lAttr in pAttributes:
			local = lAttr[0]
			distant = lAttr[1]
			min = lAttr[2] if len(lAttr) == 4 or len(lAttr) == 5 else None
			max = lAttr[3] if len(lAttr) == 4 or len(lAttr) == 5 else None
			
			if len(lAttr) == 3:
				lDescriptions.append(lAttr[2])
			elif len(lAttr) == 5:
				lDescriptions.append(lAttr[4])
			else:
				lDescriptions.append('')
			
			if distant in self.__mAttributes:
				raise Exception('setting ' + distant + ' is already associated to attribute ' + self.__mAttributes[distant].mAttributeName)
			# ensure that pLocalName exists and retrieve default value
			lDefaultValues.append(str(getattr(self,local)));
			self.__mAttributes[distant] = SettingsBase.SettingInfo(local,getattr(self,local), min, max)
			
			lNames.append(distant)
		lServiceReq = settings_store.srv.declareandgetRequest(lNames, lDefaultValues,lDescriptions)
		lResponse = self.__mServiceDeclareAndGet(lServiceReq)
		
		for (lAttr,val) in zip(pAttributes,lResponse.values):
			local = lAttr[0]
			distant = lAttr[1]
			self.__setAttrValFromStr(distant,val)
	
	def __setNumericAttr(self, pSettingInfo, pVal):
		if pSettingInfo.mMin is None or (pSettingInfo.mMin <= pVal and pVal <= pSettingInfo.mMax):
			setattr(self,pSettingInfo.mAttributeName,pVal)
		else:
			rospy.logwarn('Setting out of range %s %s [%s %s], default to %s' % (pSettingInfo.mAttributeName , str(pVal),str(pSettingInfo.mMin),str(pSettingInfo.mMax),pSettingInfo.mDefault))
			setattr(self,pSettingInfo.mAttributeName,pSettingInfo.mDefault)
	
	def __setAttrValFromStr(self, pDistant, pStrVal):
		lSettingInfo = self.__mAttributes[pDistant]
		lCurrentValue = getattr(self,lSettingInfo.mAttributeName)
		if isinstance(lCurrentValue,str):
			setattr(self,lSettingInfo.mAttributeName,str(pStrVal))
		elif isinstance(lCurrentValue,bool):
			lStrLower = pStrVal.lower()
			lNewVal = not (len(lStrLower) == 0 or lStrLower[0] == 'f' or lStrLower[0] == '0')
			setattr(self,lSettingInfo.mAttributeName, lNewVal)
		elif isinstance(lCurrentValue,int):
			self.__setNumericAttr(lSettingInfo,int(pStrVal))
		elif isinstance(lCurrentValue,long):
			self.__setNumericAttr(lSettingInfo,long(pStrVal))
		elif isinstance(lCurrentValue,float):
			self.__setNumericAttr(lSettingInfo,float(pStrVal))
		else:
			rospy.logfatal('type of ' + lSettingInfo.mAttributeName + ' is not handled')
	
	def __onSettingChangeCallback(self,pParam):
		if not pParam.name in self.__mAttributes:
			return
		self.__setAttrValFromStr(pParam.name,pParam.value)

# use this class to declare a state, states are not persistent and are not synchronized with the store
# states are used to report informations to the front-end
class StateDeclarator:
	
	def __init__(self):
		self.__mStates = dict()
		# wait until setting store is up
		rospy.wait_for_service('/settings_store/setstates',2.)
		self.__mServiceSetStates = rospy.ServiceProxy('/settings_store/setstates', settings_store.srv.setstates)
		self.__mPreviousValues = dict()
	
	def setState(self, pName, pValue):
		if pName in self.__mPreviousValues and self.__mPreviousValues[pName] == pValue:
			return
		self.__mPreviousValues[pName] = pValue
		lServiceReq = settings_store.srv.setstatesRequest([pName], [str(pValue)])
		lResponse = self.__mServiceSetStates(lServiceReq)
		
# this class will listen controller.html page to manage power status
class PowerWatchdog:
	def __init__(self, pStateDeclarator):
		self.__mStateDeclarator = pStateDeclarator
		self.__mIsPowerEnable = True
		self.setPowerEnable(False)
		self.__mGamePadSub = rospy.Subscriber('GamePadButtons', std_msgs.msg.String,self.__onGameBadButton)
		self.__mGamePadSub = rospy.Subscriber('PowerHeartBeat', std_msgs.msg.Int32,self.__onHeartBeat)
		self.__mEmergencyStopTs = None
	
		
	def setPowerEnable(self, pEnable):
		self.__setPowerEnable(pEnable,'code');
		
	def isPowerEnable(self):
		if self.__mEmergencyStopTs != None and self.__mEmergencyStopTs < datetime.datetime.now():
			self.__setPowerEnable(False,'lost heartbeat')
		return self.__mIsPowerEnable
	
	def __setPowerEnable(self, pEnable, pMsg):
		if self.__mIsPowerEnable == pEnable:
			return
		self.__mIsPowerEnable = pEnable
		self.__mStateDeclarator.setState("powerstatus", "enable" if self.__mIsPowerEnable else "disable")
		rospy.logwarn('Power is %s (source %s)' % (self.__mIsPowerEnable,pMsg))
	
	def __onGameBadButton(self,pParam):
		if '|StopPower|' in pParam.data:
			self.__setPowerEnable(False,'emergency stop')
		elif '|StartPower|' in pParam.data:
			self.__setPowerEnable(True,'power button')
		elif '|B|' in pParam.data:
			self.__setPowerEnable(False,'xbox')
			if '|Y|' in pParam.data:
				self.__setPowerEnable(True,'xbox')
			
	def __onHeartBeat(self, pParam):
		if pParam.data < 0:
			self.__mEmergencyStopTs = None
			self.__setPowerEnable(False,'heartbeat page was closed')
		elif pParam.data == 0:
			self.__mEmergencyStopTs = datetime.datetime.now()
			self.__setPowerEnable(False,'emergency stop')
		else:
			self.__mEmergencyStopTs = datetime.datetime.now() + datetime.timedelta(seconds=pParam.data)
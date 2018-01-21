#!/usr/bin/env python
# coding: utf-8

import rospy
import settings_store.msg
import settings_store.srv

class SettingsBase:

	def __init__(self):
		self.__mAttributes = {}
		rospy.Subscriber('settings_store/Change',settings_store.msg.change,self)
		rospy.wait_for_service('/settings_store/declareandget',2.)
		self.__mServiceDeclareAndGet = rospy.ServiceProxy('/settings_store/declareandget', settings_store.srv.declareandget)
		
	def __setAttrValFromStr(self, pName, pStrVal):
		lCurrentValue = getattr(self,pName)
		if isinstance(lCurrentValue,str):
			setattr(self,pName,str(pStrVal))
		elif isinstance(lCurrentValue,int):
			setattr(self,pName,int(pStrVal))
		elif isinstance(lCurrentValue,long):
			setattr(self,pName,long(pStrVal))
		elif isinstance(lCurrentValue,float):
			setattr(self,pName,float(pStrVal))
		elif isinstance(lCurrentValue,bool):
			setattr(self,pName,bool(pStrVal))
		else:
			rospy.logfatal('type of ' + pName + ' is not handled')
	
	def __call__(self,pParam):
		if not pParam.name in self.__mAttributes:
			return
		lLocalName = self.__mAttributes[pParam.name]
		self.__setAttrValFromStr(lLocalName,pParam.value)
		
	# this method will register given attributes to sync them with settings store
	# pAttributes is a list of pairs (class attribute name, settings store key name)
	def registerAttributes(self, pAttributes):
		lNames = []
		lDefaultValues = []
		
		for (local,distant) in pAttributes:
			if distant in self.__mAttributes:
				raise Exception('setting ' + distant + ' is already associated to attribute ' + self.__mAttributes[distant])
			# ensure that pLocalName exists and retrieve default value
			lDefaultValues.append(str(getattr(self,local)));
			self.__mAttributes[distant] = local
			
			lNames.append(distant)
		lServiceReq = settings_store.srv.declareandgetRequest(lNames, lDefaultValues)
		lResponse = self.__mServiceDeclareAndGet(lServiceReq)
		
		for ((local,distant),val) in zip(pAttributes,lResponse.values):
			self.__setAttrValFromStr(local,val)
	
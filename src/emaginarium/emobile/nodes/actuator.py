#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import pigpio
import time
import emobile.msg
import os
import emaginarium_common.msg
import std_msgs
from settings_store import settings_store_client


class ActuatorSettings(settings_store_client.SettingsBase):

	def __init__(self):
		settings_store_client.SettingsBase.__init__(self)
		self.kTh=0.3
		self.registerAttributes([
			('kTh','actuator/kTh','kTh'),
			])

class Actuator(object):
	def __init__(self,pinT,pinS):
		self.__mStateDeclarator = settings_store_client.StateDeclarator()
		self.throttles = 0
		self.steering = 0
		self.settings = ActuatorSettings()
		self.mGoalThrottle=0 #-1 pour marche arriere max 0 arret 1 marche avant max
		self.mGoalSteering=0
		self.__mGpio = None
		try:
			self.__mGpio = pigpio.pi()
		except:
			self.__mGpio = None
		self.pinT=pinT
		self.pinS=pinS
		self.__mIsEnable = False
		self.__mStateDeclarator.setState("actuator/enable","disable")
				
		time.sleep(5)
		if self.__mGpio is None:
			self.__mStateDeclarator.setState("init/actuator",False)
			rospy.logerr('Fail to open gpio, actuator will not work properly')
		else:
			self.__mStateDeclarator.setState("init/actuator",True)
			rospy.logwarn('Powertrain is ready')

	def updateThrottleTarget(self, param):
		self.mGoalThrottle = min(max(-1.,param.throttle),1.)
		if self.mGoalThrottle != param.throttle:
			rospy.logerr('Goal speed out of range %f' % (param.throttle))
		self.updateThrottle()
		
	def updateThrottle(self):
		self.mGoalThrottle = min(max(-1.,self.mGoalThrottle),1.)

		if self.__mIsEnable:
			self.throttles = (1.48 - self.mGoalThrottle * self.settings.kTh)*1000
		else:
			self.throttles = 1520
		
		if self.__mGpio is not None:
			self.__mGpio.set_servo_pulsewidth(self.pinT, self.throttles)

	def updateSteeringTarget(self, param):
		self.mGoalSteering = min(max(-1.,param.steering),1.)
		if self.mGoalSteering != param.steering:
			rospy.logerr('Goal steering out of range %f' % (param.steering))
		self.updateSteering()

	def updateSteering(self):
		self.mGoalSteering = min(max(-1.,self.mGoalSteering),1.)
		self.steering= (1.49-0.25*self.mGoalSteering)*1000
		if self.__mGpio is not None:
			self.__mGpio.set_servo_pulsewidth(self.pinS, self.steering)
		

 	def updatexbox (self,param):
		if '|B|' in param.data:
			lEnable = False
			if '|Y|' in param.data:
				lEnable = True
			if self.__mIsEnable != lEnable:
				self.__mStateDeclarator.setState("actuator/enable",('enable' if lEnable else 'disable'))
				rospy.logwarn('Actuator is %s' % ('enable' if lEnable else 'disable'))
				self.__mIsEnable = lEnable
			self.updateThrottle()


		
if __name__ == "__main__":	
	os.getcwd()
	rospy.init_node('actuator')

	lActuator = Actuator(rospy.get_param('/actuator/pinT'),rospy.get_param('/actuator/pinS'))
	sRosSuscriberSpeed = rospy.Subscriber('GamePadButtons', std_msgs.msg.String,lActuator.updatexbox)
	sRosSuscriberThrottle = rospy.Subscriber('emobile/CommandThrottle', emobile.msg.CommandThrottle,lActuator.updateThrottleTarget)
	sRosSuscriberSteering = rospy.Subscriber('emobile/CommandSteering', emobile.msg.CommandSteering,lActuator.updateSteeringTarget)
	rospy.spin()

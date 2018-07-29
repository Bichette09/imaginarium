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
		self.throttles = 0
		self.steering = 0
		self.settings = ActuatorSettings()
		self.mGoalThrottles=0 #-1 pour marche arriere max 0 arret 1 marche avant max
		self.__mGpio = None
		try:
			self.__mGpio = pigpio.pi()
		except:
			self.__mGpio = None
		self.pinT=pinT
		self.pinS=pinS
		self.__mIsEnable = False
		time.sleep(5)
		if self.__mGpio is None:
			rospy.logerr('Fail to open gpio, actuator will not work properly')
		else:
			rospy.loginfo('Powertrain is ready')

	def updateThrottlesTarget(self, param):
		self.mGoalThrottles = min(max(-1.,param.throttle),1.)
		if self.mGoalThrottles != param.throttle:
			rospy.logerr('Goal speed out of range %f' % (param.throttle))
		self.updateThrottles()
		
	def updateThrottles(self):
		self.mGoalThrottles = min(max(-5.,self.mGoalThrottles),5.)

		if self.__mIsEnable:
			self.throttles = (1.48 - self.mGoalThrottles * self.settings.kTh)*1000
		else:
			self.throttles = 1520
		
		if self.__mGpio is not None:
			self.__mGpio.set_servo_pulsewidth(self.pinT, self.throttles)

	def updateSteering(self):
		if self.__mGpio is None:
			return
		self.__mGpio.set_servo_pulsewidth(self.pinS, self.steering)
		

 	def updatexbox (self,param):
		if '|B|' in param.data:
			self.__mIsEnable = False
			if '|Y|' in param.data:
				self.__mIsEnable = True				
					
		self.updateThrottles()

	def updatestick (self,param):
		self.steering= (1.49-0.25*param.data[0])*1000
		self.updateSteering()

		
if __name__ == "__main__":	
	os.getcwd()
	rospy.init_node('actuator')

	lActuator = Actuator(rospy.get_param('/actuator/pinT'),rospy.get_param('/actuator/pinS'))	
	sRosSuscriberSpeed = rospy.Subscriber('GamePadButtons', emaginarium_common.msg.GamePadButtons,lActuator.updatexbox)
	sRosSuscriberSteering = rospy.Subscriber('GamePadSticks', std_msgs.msg.Float32MultiArray,lActuator.updatestick)
	sRosSuscriberThrottle = rospy.Subscriber('emobile/CommandThrottle', emobile.msg.CommandThrottle,lActuator.updateThrottlesTarget)
	rospy.spin()

#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import pigpio
import time
import emobile.msg
from settings_store import settings_store_client
import os

class Actuator(object):
	def __init__(self,pinT,pinS,settings):
		self.throttles = 0
		self.steering = 0
		self.__mGpio = pigpio.pi()
		self.pinT=pinT
		self.pinS=pinS
		self.settings = settings 
		time.sleep(5)
		print('Powertrain is ready')
	
		
	def updateThottles(self):
			self.__mGpio.set_servo_pulsewidth(self.pinT, self.throttles)

	def updateSteering(self):
			self.__mGpio.set_servo_pulsewidth(self.pinS, self.steering)

 	def updatexbox (self,param):
		if '|B|' in param.data:
			self.throttles = 1520
			if '|Y|' in param.data:
				self.throttles = 1540
					
		self.updateThottles()

	def updatestick (self,param):
		self.steering= (1.49-0.25*param.data[0])*1000
		self.updateSteering()

class ActuatorSettings(settings_store_client.SettingsBase):

	def __init__(self):
		settings_store_client.SettingsBase.__init__(self)
		self.kp=1
		self.registerAttributes([
			('kp','command/Kp')
			])
		
if __name__ == "__main__":	
	os.getcwd()
	rospy.init_node('actuator')

	#creation instance settings pour les parametres modifiables
	lSettings = ActuatorSettings()

	lActuator = Actuator(rospy.get_param('/actuator/pinT'),rospy.get_param('/actuator/pinS'),lSettings)	
	sRosSuscriberSpeed = rospy.Subscriber('GamePadButtons', emaginarium.msg.GamePadButtons,lActuator.updatexbox)
	sRosSuscriberSteering = rospy.Subscriber('GamePadSticks', emaginarium.msg.GamePadSticks,lActuator.updatestick)
	

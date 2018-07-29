#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import pigpio
import time
import emobile.msg
import os
import emaginarium_common.msg
import std_msgs

class Actuator(object):
	def __init__(self,pinT,pinS):
		self.throttles = 0
		self.steering = 0
		self.__mGpio = None
		try:
			self.__mGpio = pigpio.pi()
		except:
			self.__mGpio = None
		self.pinT=pinT
		self.pinS=pinS
		time.sleep(5)
		if self.__mGpio is None:
			rospy.logerr('Fail to open gpio, actuator will not work properly')
		else:
			rospy.loginfo('Powertrain is ready')
	
		
	def updateThottles(self):
		if self.__mGpio is None:
			return
		self.__mGpio.set_servo_pulsewidth(self.pinT, self.throttles)

	def updateSteering(self):
		if self.__mGpio is None:
			return
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

		
if __name__ == "__main__":	
	os.getcwd()
	rospy.init_node('actuator')

	lActuator = Actuator(rospy.get_param('/actuator/pinT'),rospy.get_param('/actuator/pinS'))	
	sRosSuscriberSpeed = rospy.Subscriber('GamePadButtons', emaginarium_common.msg.GamePadButtons,lActuator.updatexbox)
	sRosSuscriberSteering = rospy.Subscriber('GamePadSticks', std_msgs.msg.Float32MultiArray,lActuator.updatestick)
	rospy.spin()

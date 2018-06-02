#!/usr/bin/env python
# coding: utf-8

import imaginarium_core.msg
import rospy
import os
import time
import imaginariumlib

class Speed(object):

	def __init__(self, pPinA, pPinB):
		self.__mTickCounter = 0
		self._rotaryEncoder = imaginariumlib.RotaryEncoder(pPinA,pPinB)
		self._rotaryEncoder.when_rotated = self.onCountChange
		self.__mPreviousTimeStamp = None

	def onCountChange(self, pValue):
		self.__mTickCounter = self.__mTickCounter + pValue
		
	def readSpeed(self):
		lDistance = 0
		lSpeed = 0
		lTickCount = self.__mTickCounter
			
		self.__mTickCounter = 0
		lDistance = lTickCount * 0.01263
		lNewTimeStamp = time.time()
		if self.__mPreviousTimeStamp is not None:
			lSpeed = lDistance / (lNewTimeStamp - self.__mPreviousTimeStamp)
		self.__mPreviousTimeStamp = lNewTimeStamp
		
		return lSpeed
			
if __name__ == "__main__":
	
	os.getcwd()
	rospy.init_node('speed')

	
	sRosPublisher = rospy.Publisher('imaginarium_core/Speed', imaginarium_core.msg.Speed, queue_size=5)
	lSpeedReader = Speed(rospy.get_param('/speed/pinA'),rospy.get_param('/speed/pinB'))

	while not rospy.core.is_shutdown():
	
		lSpeed = lSpeedReader.readSpeed()
		time.sleep(0.05)
		# Message publication
		sRosPublisher.publish(imaginarium_core.msg.Speed(lSpeed))

#!/usr/bin/env python
# coding: utf-8

import imaginarium_core.msg
import rospy
import os
import time

class Speed(object):

  def __init__(self):
	self.__mTickCounter = 0
	self._rotaryEncoder = RotaryEncoder(24,23)
	self._rotaryEncoder.when_rotated = self.onCountChange
	self.__mPreviousTimeStamp = None		
      
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
  	lSpeedReader = Speed()

	while not rospy.core.is_shutdown():
	
		lDistance = lSpeedReader.readSpeed()
			
		# Message publication
		sRosPublisher.publish(imaginarium_core.msg.Speed(lSpeed))

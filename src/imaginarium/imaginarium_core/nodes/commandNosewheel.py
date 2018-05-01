#!/usr/bin/env python
# coding: utf-8

import imaginarium_core.msg
import rospy
import os
import serial

class CommandNosewheel(object):
	
	def __init__(self):
		self.speed = 0
		self.antenna = false
		self.ultrason= [0]*10
	
 	def updateSpeed(self,param):
		self.speed = param.speed
	
	def updateUltrason(self,param):
		self.ultrason = param.

if __name__ == "__main__":
	
	os.getcwd()
	rospy.init_node('commandNosewheel')

	
	sRosPublisher = rospy.Publisher('imaginarium_core/CommandNosewheel', imaginarium_core.msg.CommandNosewheel, )
	lCommandNosewheel = CommandNosewheel(rospy.get_param('/commandNosewheel/serialPort'))
	sRosSuscriberUltra = rospy.Subscriber('imaginarium_core/Ultrasound', imaginarium_core.msg.Ultrasound,)
	sRosSuscriberSpeed = rospy.Subscriber('imaginarium_core/Speed', imaginarium_core.msg.Speed,lCommandNosewheel.updateSpeed)
	sRosSuscriberAntenna = rospy.Subscriber('imaginarium_core/Antenna', imaginarium_core.msg.Antenna, )
	
	while not rospy.core.is_shutdown():
	

	
		# Message publication
	sRosPublisher.publish(imaginarium_core.msg.CommandNosewheel(lNoseWheelAngle))

#!/usr/bin/env python
# coding: utf-8


import emobile.msg
import emaginarium_common.msg
import std_msgs
import rospy
import os
import math
from settings_store import settings_store_client
import time
import numpy as np


class CommandSettings(settings_store_client.SettingsBase):

	def __init__(self):
		settings_store_client.SettingsBase.__init__(self)
		self.k=1.
		self.registerAttributes([
			('k','command/K','Gain'),
			])

class ControlLaw():
	
	def __init__(self):
		self.throttleGoal = 0
		self.steeringGoal =0

	def updatestickThrottle (self,param):
		self.throttleGoal= param.data[3]

		
if __name__ == "__main__":
	
	os.getcwd()
	rospy.init_node('command')

	#creation instance settings pour les parametres modifiables
	lSettings = CommandSettings()
	
	sRosPublisherSteering = rospy.Publisher('emobile/CommandSteering', emobile.msg.CommandSteering, queue_size=5)
	sRosPublisherThrottle = rospy.Publisher('emobile/CommandThrottle', emobile.msg.CommandThrottle, queue_size=5)
	
	lControlLaw = ControlLaw()
	sRosSuscriberThrottle = rospy.Subscriber('GamePadSticks', std_msgs.msg.Float32MultiArray,lControlLaw.updatestickThrottle)

	
	
	while not rospy.core.is_shutdown():
		
		# we want 100Hz reresh rate
		time.sleep(0.01)
	
		sRosPublisherThrottle.publish(emobile.msg.CommandThrottle(lControlLaw.throttleGoal))
		


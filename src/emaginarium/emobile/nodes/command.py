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
		self.kprop=1.
		self.kprec=1.
		self.registerAttributes([
			('kprop','command/Kprop','Gain of prop command'),
			('kprec','command/Kprec','Gain of pre command')
			])

class ControlLaw():
	
	def __init__(self):
		self.throttleGoal = 0
		self.steeringGoal =0
		self.ledarDist = [0]*24
		self.ledarDistX0 = [0]*24
		self.ledarDistY0 = [0]*24

	def updatestick(self,param):
		self.throttleGoal= param.data[3]
		self.steeringGoal= param.data[0]
		
	def onNewLedar(self,param):
		self.ledarDist = param.distance
		self.ledarDistX0 = param.x1
		self.ledarDistY0 = param.y1

	def computeAngleToBorders(self):
		vU8_X=self.ledarDistX0[0:8]
		vU8_Y=self.ledarDistY0[0:8]
		
		resFit = 0.
		try:
			resFit = np.polyfit(np.array(vU8_X),np.array(vU8_Y),1)
			rospy.logwarn(resFit)
			resFit = resFit[0]
			
		except:
			pass
		
		
		lAngle=math.atan(resFit)*57.3
		if not np.isfinite(lAngle):
			return None
		if self.ledarDistY0[7] > self.ledarDistY0[0]:
			lAngle = -lAngle
		return lAngle
		
		
if __name__ == "__main__":
	
	os.getcwd()
	rospy.init_node('command')

	#creation instance settings pour les parametres modifiables
	lSettings = CommandSettings()
	
	sRosPublisherSteering = rospy.Publisher('emobile/CommandSteering', emobile.msg.CommandSteering, queue_size=5)
	sRosPublisherThrottle = rospy.Publisher('emobile/CommandThrottle', emobile.msg.CommandThrottle, queue_size=5)
	
	lControlLaw = ControlLaw()
	sRosSuscriberThrottle = rospy.Subscriber('GamePadSticks', std_msgs.msg.Float32MultiArray,lControlLaw.updatestick)
	sRosSuscriberLedar = rospy.Subscriber('/pointcloud', emobile.msg.PointCloud,lControlLaw.onNewLedar)
	
	
	while not rospy.core.is_shutdown():
		
		# we want 100Hz reresh rate
		time.sleep(0.01)
		lFinalAngle = lControlLaw.computeAngleToBorders()
		sRosPublisherThrottle.publish(emobile.msg.CommandThrottle(lControlLaw.throttleGoal))
		sRosPublisherSteering.publish(emobile.msg.CommandSteering(lControlLaw.steeringGoal,lFinalAngle))


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
		self.L=2.
		self.D=0.75
		self.registerAttributes([
			('L','command/L','Prediction distance'),
			('D','command/D','Commanded distance from the side')
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
		#self.steeringGoal= param.data[0]
		
	def onNewLedar(self,param):
		self.ledarDist = param.distance
		self.ledarDistX0 = param.x1
		self.ledarDistY0 = param.y1

	def computeAngleToBorders(self):
		# remove null measures
		vU8_X = []
		vU8_Y = []
		for (x,y,d) in zip(self.ledarDistX0[0:8],self.ledarDistY0[0:8],self.ledarDist):
			if d > 0.001:
				# ignore measures that are too close
				vU8_X.append(x)
				vU8_Y.append(y)
		
		lA = 0.
		lB = 0.
		lAngle=0.
		try:
			if len(vU8_X) == 0:
				raise Exception()
			resFit = np.polyfit(np.array(vU8_X),np.array(vU8_Y),1)
			lA = resFit[0]
			lB = resFit[1]
			lAngle=math.atan(lA)*57.3
		except:
			pass
				
		if not np.isfinite(lAngle):
			return None
		return (lAngle,lA,lB)

		
if __name__ == "__main__":
	
	os.getcwd()
	rospy.init_node('command')

	#creation instance settings pour les parametres modifiables
	lSettings = CommandSettings()
	
	sRosPublisherSteering = rospy.Publisher('emobile/CommandSteering', emobile.msg.CommandSteering, queue_size=1)
	sRosPublisherThrottle = rospy.Publisher('emobile/CommandThrottle', emobile.msg.CommandThrottle, queue_size=1)
	sRosPublisherDebug2dPrimitives = rospy.Publisher('emobile/Debug2dPrimitive', emaginarium_common.msg.Debug2dPrimitive, queue_size=1)
	
	lControlLaw = ControlLaw()
	sRosSuscriberThrottle = rospy.Subscriber('GamePadSticks', std_msgs.msg.Float32MultiArray,lControlLaw.updatestick)
	sRosSuscriberLedar = rospy.Subscriber('/pointcloud', emobile.msg.PointCloud,lControlLaw.onNewLedar)
	
	lWheelAnglec = 0.
	
	while not rospy.core.is_shutdown():
		
		# we want 100Hz reresh rate
		time.sleep(0.01)
		(lFinalAngle,lA,lB) = lControlLaw.computeAngleToBorders()


		
		# Predictive command
		#objectivePointX=lSettings.L+lSettings.D*math.cos(lFinalAngle/57.3)
		objectivePointX=lSettings.L
		#objectivePointY=lA*lSettings.L+lB-lSettings.D*math.sin(lFinalAngle/57.3)
		objectivePointY=lA*lSettings.L+lB+lSettings.D
		lWheelAnglec=math.atan(objectivePointY/(objectivePointX-0.265))*57.3
		lWheelAnglec = max(-35,min(35,lWheelAnglec))
		
		#Command normalisation
		lControlLaw.steeringGoal=lWheelAnglec/35

		sRosPublisherThrottle.publish(emobile.msg.CommandThrottle(lControlLaw.throttleGoal))
		sRosPublisherSteering.publish(emobile.msg.CommandSteering(lControlLaw.steeringGoal))
		sRosPublisherDebug2dPrimitives.publish(emaginarium_common.msg.Debug2dPrimitive('sideline','line','red',[lA,lB]))
		sRosPublisherDebug2dPrimitives.publish(emaginarium_common.msg.Debug2dPrimitive('targetPoint','circle','green',[objectivePointX,objectivePointY,0.05]))

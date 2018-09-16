#!/usr/bin/env python
# coding: utf-8

import emaginarium.msg
import rospy
import os
import serial
import math
import pypot.dynamixel
# import pigpio
from settings_store import settings_store_client
import time
import numpy as np

class Dynamixel(object):
	
	def __init__(self,serialPort):
		#Servomoteur init
		self.__dxl_io = pypot.dynamixel.DxlIO(serialPort)
		if self.__dxl_io is None:
			rospy.logerr('Servo is disabled, fail to create DxlIO object')
		# self.__mGpio = pigpio.pi()
		# if self.__mGpio is None or not self.__mGpio.connected:
			# print('Powertrain is disabled, fail to create GPIO object')
			# self.__mGpio = None
		time.sleep(5)
		self.__dxl_io.set_pid_gain({1:(20,5,0)})
		# print('Powertrain is ready')
	
 
	def setWheelAngle(self,angle):
		if self.__dxl_io is not None:
			try:
				self.__dxl_io.set_goal_position({1:angle})
			except:
				pass
	
	def setCompliant(self):
		if self.__dxl_io is not None:
			try:
				self.__dxl_io.disable_torque({1:True})
			except:
				pass

class CommandNosewheelSettings(settings_store_client.SettingsBase):

	def __init__(self):
		settings_store_client.SettingsBase.__init__(self)
		self.kprop=1.
		self.kprec=1.
		self.registerAttributes([
			('kprop','commandNosewheel/Kprop','Gain of prop command'),
			('kprec','commandNosewheel/Kprec','Gain of pre command')
			])

class ControlLaw():
	
	def __init__(self):
		self.speed = 0
		self.ultrasonDist = [0]*10
		self.ultrasoundDistX0 = [0]*10
		self.ultrasoundDistY0 = [0]*10
		self.lidarDist = [0]
		self.lidarDistX0 = [0]
		self.lidarDistY0 = [0]
		self.leftAngle = 0.
		self.rightAngle = 0.
		self.leftDistance = 0.
		self.rightDistance = 0.
		self.leftAngleValid = 0.
		self.rightAngleValid = 0.
		self.leftDistanceValid = 0.
		self.rightDistanceValid = 0.
		
	def onNewSpeed(self,param):
		self.speed = param.speed
	
	def onNewUltrasound(self,param):
		self.ultrasonDist = param.distance
		self.ultrasoundDistX0 = param.x0
		self.ultrasoundDistY0 = param.y0

	def onNewTinyLidar(self,param):
		self.lidarDist = param.distance
		self.lidarDistX0 = param.x0
		self.lidarDistY0 = param.y0
	
	def onNewSides(self,param):
		self.leftAngle = None
		self.rightAngle = None
		self.leftDistance = None
		self.rightDistance = None
		if param.leftAngleValid:
			self.leftAngle = param.leftAngle
		if param.rightAngleValid:
			self.rightAngle = param.rightAngle
		if param.leftDistanceValid:
			self.leftDistance = param.leftDistance
		if param.rightDistanceValid:
			self.rightDistance = param.rightDistance
	
	def updateAntenna(self,param):
		self.antennaStatus = param.status
		
if __name__ == "__main__":
	
	os.getcwd()
	rospy.init_node('commandNosewheel')

	#creation instance settings pour les parametres modifiables
	lSettings = CommandNosewheelSettings()
	
	sRosPublisher = rospy.Publisher('emaginarium/CommandNosewheel', emaginarium.msg.CommandNosewheel, queue_size=5)
	sRosSuscriberSpeedTarget = rospy.Publisher('emaginarium/SpeedTarget', emaginarium.msg.SpeedTarget, queue_size=5)
	
	lDynamixel = Dynamixel(rospy.get_param('/commandNosewheel/serialPort'))
	lControlLaw = ControlLaw()
	sRosSuscriberUltra = rospy.Subscriber('emaginarium/Ultrasound', emaginarium.msg.Ultrasound,lControlLaw.onNewUltrasound)
	sRosSuscriberSpeed = rospy.Subscriber('emaginarium/Speed', emaginarium.msg.Speed,lControlLaw.onNewSpeed)
	sRosSuscriberLiDar = rospy.Subscriber('emaginarium/TinyLidar', emaginarium.msg.TinyLidar,lControlLaw.onNewTinyLidar)
	sRosSuscriberSides = rospy.Subscriber('/emaginarium/Sides', emaginarium.msg.Sides,lControlLaw.onNewSides)
	
	
	#init
	cptDoor = 0
	lWheelAngle = 0.
	lFrontDist = 0.
	lRightDist = 0.
	lLeftDist = 0.
	lTimeInt=time.time()
	ierreur=0.
	lProp=0.
	lPrec=0.
	lLeftAngle=0.
	lRightAngle=0.
	lFinalAngle=0.
	lLastSpeedTarget=0.
	lPrec = 0.
	
	while not rospy.core.is_shutdown():
		
		# we want 100Hz reresh rate
		time.sleep(0.01)
		lProp = 0.
		# personne devant => precommande only
		if lControlLaw.ultrasonDist[4] == 0. and lControlLaw.ultrasonDist[5] == 0.:
			lProp = 0.
		else:
			if lControlLaw.ultrasonDist[2] == 0. and lControlLaw.ultrasonDist[7] == 0.:
				lProp = 0.
			elif lControlLaw.ultrasonDist[2] == 0. and lControlLaw.ultrasonDist[7] > 0.:
				lProp = 40
			elif lControlLaw.ultrasonDist[2] > 0. and lControlLaw.ultrasonDist[7] == 0.:
				lProp = -40
			elif lControlLaw.ultrasonDist[2] > 0. and lControlLaw.ultrasonDist[7] > 0.:
				if lControlLaw.ultrasonDist[2] > lControlLaw.ultrasonDist[7]:
					lProp = 40
				else:
					lProp = -40
		
		print lProp,lControlLaw.ultrasonDist[2]+lControlLaw.ultrasonDist[5]
			
		# compute distance to control from the side
		if lControlLaw.leftDistance is not None and lControlLaw.rightDistance is not None:
			Obj = (lControlLaw.leftDistance + lControlLaw.rightDistance-0.360)/2.0
		else: 
			Obj = (1.5-0.36)/2.0
		
		# compute position of the objective in the robot frame (y coordinate with respect to the two sides)
		avPhase = 1.5
		if lControlLaw.rightAngle is not None:
			yRight=Obj-(lControlLaw.rightDistance)+avPhase*np.tan(lControlLaw.rightAngle*np.pi/180.0)
		if lControlLaw.leftAngle is not None:
			yLeft=-Obj+(lControlLaw.leftDistance)+avPhase*np.tan(lControlLaw.leftAngle*np.pi/180.0)
		
		if lControlLaw.leftAngle is not None and lControlLaw.rightAngle is not None:
		# the two angles are valid, take the closest one
			if lControlLaw.leftDistance<lControlLaw.rightDistance:
				y = yLeft
			else:
				y = yRight
		# if only left or right are valid, take it
		elif lControlLaw.leftAngle is not None and lControlLaw.rightAngle is None:
			y = yLeft
		elif lControlLaw.leftAngle is None and lControlLaw.rightAngle is not None:
			y = yRight
		else:
		# all right
			y = 0.0

		inov = 0.5 # innovation filter
		lPrec = (1-inov)*lPrec+inov*(-180.0*np.arctan(y/avPhase)/np.pi)
		#print lPrec
			
		lWheelAngle = lProp+lPrec
		lWheelAngle = max(-45,min(45,lWheelAngle))

		lDynamixel.setWheelAngle(lWheelAngle)
		# Message publication
		sRosPublisher.publish(emaginarium.msg.CommandNosewheel(lWheelAngle,False,lProp,lPrec,lFinalAngle,lLeftAngle,lRightAngle))
		sRosSuscriberSpeedTarget.publish(emaginarium.msg.SpeedTarget(lLastSpeedTarget))
		
	# Message publication en cas de fermeture de la node on envoie un status
	sRosPublisher.publish(emaginarium.msg.CommandNosewheel(lWheelAngle,True,lProp,lPrec,lFinalAngle,lControlLaw.leftAngle,lControlLaw.rightAngle))
	lDynamixel.setCompliant()
	rospy.sleep(0.5)
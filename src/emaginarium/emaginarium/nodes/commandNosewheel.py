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
		# print('Powertrain is ready')
	
 
	def setWheelAngle(self,angle):
		if self.__dxl_io is not None:
			try:
				self.__dxl_io.set_goal_position({1:angle})
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
		# retrait du capteur arriere (cm)
		self.rd = 1.9
		self.ld = 1.6
		# distance entre le capteur arriere et avant (cm)		
		self.rL = 37.5
		self.lL = 37.5
		
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
	
	def updateAntenna(self,param):
		self.antennaStatus = param.status
	
	# this method will compute angle with a border based on side sensors
	# if an error occurs it will return None
	def __computeSideAngle(self, pPtFront, pPtBack):
		if self.ultrasonDist[pPtFront] <= 0. or self.ultrasonDist[pPtBack] <= 0.:
			return None
		try:
			# let A be the measure point closest to the front
			lA = np.array([self.ultrasoundDistX0[pPtFront],self.ultrasoundDistY0[pPtFront]])
			# let O be the other measured point
			lO = np.array([self.ultrasoundDistX0[pPtBack],self.ultrasoundDistY0[pPtBack]])
			# let B as O plus dir vector (y)
			lB = lO + np.array([0.,1.])
			lOA = lA - lO
			lOB = lB - lO
			lDot = np.dot(lOA,lOB)
			lAngle = math.acos(lDot / np.linalg.norm(lOA)) * 57.3
			if not np.isfinite(lAngle):
				return None
			if self.ultrasoundDistX0[pPtFront] > self.ultrasoundDistX0[pPtBack]:
				lAngle = -lAngle
			return lAngle
		except:
			return None
		
	
	def computeAngleToBorders(self):
		lLeftAngle = self.__computeSideAngle(9,8)
		lRightAngle = self.__computeSideAngle(0,1)
		
		lFinalAngle = lLeftAngle
		if lFinalAngle is None:
			lFinalAngle = lRightAngle
		elif lRightAngle is not None and math.fabs(lFinalAngle) > math.fabs(lRightAngle):
			# both are angle are valid, take the smallest one in absolute value
			lFinalAngle = lRightAngle
		
		if lFinalAngle is None:
			lFinalAngle = 0.
		if lLeftAngle is None:
			lLeftAngle = 99.
		if lRightAngle is None:
			lRightAngle = 99.
		#  rospy.logwarn('%s %s %s' %(str(lFinalAngle),str(lLeftAngle),str(lRightAngle)))
		return (lFinalAngle,lLeftAngle,lRightAngle)
		
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
	
	while not rospy.core.is_shutdown():
		
		# we want 100Hz reresh rate
		time.sleep(0.01)
		

		if lControlLaw.ultrasonDist is not None:
			lRightDist = 0.
			for i in [0,2,3]:		
				if	lControlLaw.ultrasonDist[i] > 0:
					if lRightDist > 0:
						lRightDist = min(lRightDist,lControlLaw.ultrasonDist[i])
					else:
						lRightDist = lControlLaw.ultrasonDist[i]
			lLeftDist = 0.
			for i in [9,7,6]:		
				if	lControlLaw.ultrasonDist[i] > 0:
					if lLeftDist > 0:
						lLeftDist = min(lLeftDist,lControlLaw.ultrasonDist[i])
					else:
						lLeftDist = lControlLaw.ultrasonDist[i]
				
			lFrontDist = lControlLaw.lidarDist[0];
			if lFrontDist <= 600 and lFrontDist>0:
				lLastSpeedTarget = 0.
			else:
				lLastSpeedTarget = 1.
			
			(lFinalAngle, lLeftAngle,lRightAngle) = lControlLaw.computeAngleToBorders()
				
			# compute angle			
			lWheelAngle = 0.
			if lLeftDist != 0 and lRightDist != 0:
			#Angle control law
				erreur= (lRightDist-lLeftDist)
				#Proportional term
				lProp= erreur* lSettings.kprop
				#precommande (moyenne des directions des bords)
				lPrec = lFinalAngle*lSettings.kprec
				# synthese of all terms
				lWheelAngle = lPrec + lProp

				
				lWheelAngle = max(-30,min(30,lWheelAngle))
			elif lLeftDist == 0 and lRightDist != 0:
				lWheelAngle = -30
			elif lLeftDist != 0 and lRightDist == 0:
				lWheelAngle = 30
			
		lDynamixel.setWheelAngle(lWheelAngle)
		# Message publication
		sRosPublisher.publish(emaginarium.msg.CommandNosewheel(lWheelAngle,False,lProp,lPrec,lFinalAngle,lLeftAngle,lRightAngle))
		sRosSuscriberSpeedTarget.publish(emaginarium.msg.SpeedTarget(lLastSpeedTarget))
		
	# Message publication en cas de fermeture de la node on envoie un status
	sRosPublisher.publish(emaginarium.msg.CommandNosewheel(lWheelAngle,True,lProp,lPrec,lFinalAngle,lLeftAngle,lRightAngle))

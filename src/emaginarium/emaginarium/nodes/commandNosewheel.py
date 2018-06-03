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
		self.kp=1.
		self.registerAttributes([
			('kp','commandNosewheel/Kp')
			])

class ControlLaw():
	
	def __init__(self):
		self.speed = 0
		self.ultrasonDist = [0]*10
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
	
	
	#init
	cptDoor = 0
	lLastAngle = 0.
	lFrontDist = 0.
	lRightDist = 0.
	lLeftDist = 0.
	lTimeInt=time.time()
	ierreur=0.
	lProp=0.
	lPrec=0.
	lLeftAngle=0.
	lRightAngle=0.
	lLastSpeedTarget=0.
	
	while not rospy.core.is_shutdown():
		
		# we want 100Hz reresh rate
		time.sleep(0.01)
		

		if lControlLaw.ultrasonDist is not None:
			lRightDist = 0.
			for i in range(0,4):		
				if	lControlLaw.ultrasonDist[i] > 0:
					if lRightDist > 0:
						lRightDist = min(lRightDist,lControlLaw.ultrasonDist[i])
					else:
						lRightDist = lControlLaw.ultrasonDist[i]
			lLeftDist = 0.
			for i in range(6,10):		
				if	lControlLaw.ultrasonDist[i] > 0:
					if lLeftDist > 0:
						lLeftDist = min(lLeftDist,lControlLaw.ultrasonDist[i])
					else:
						lLeftDist = lControlLaw.ultrasonDist[i]
				
			lFrontDist = 0.
			for i in range(3,7):		
				if	lControlLaw.ultrasonDist[i] > 0:
					if lFrontDist > 0:
						lFrontDist = min(lFrontDist,lControlLaw.ultrasonDist[i])
					else:
						lFrontDist = lControlLaw.ultrasonDist[i]

			# angle between robot and runway
			lRightAngle = 0.
			if lControlLaw.ultrasonDist[1]>0. and lControlLaw.ultrasonDist[0]>0.:
				lRightAngle = -1.5-math.atan((lControlLaw.ultrasonDist[1]-(lControlLaw.ultrasonDist[0]+lControlLaw.rd))/lControlLaw.rL)*180/math.pi
				lLeftAngle = 0.
				if lControlLaw.ultrasonDist[8]>0. and lControlLaw.ultrasonDist[9]>0.:
					lLeftAngle =  math.atan((lControlLaw.ultrasonDist[8]-(lControlLaw.ultrasonDist[9]+lControlLaw.ld))/lControlLaw.lL)*180/math.pi


				#print "rightAngle:"+str(lRightAngle)+"\t leftAngle:"+str(lLeftAngle)
				#print "Dist8:"+str(lCommandNosewheel.ultrasonDist[8])+"\t Dist9:"+str(lCommandNosewheel.ultrasonDist[9])
				#print "Dist1:"+str(lCommandNosewheel.ultrasonDist[1])+"\t Dist0:"+str(lCommandNosewheel.ultrasonDist[0])

				
				# compute angle			
				lLastAngle = 0.
				if lLeftDist != 0 and lRightDist != 0:
				#Angle control law
					erreur= (lRightDist-lLeftDist)
					#Proportional term
					lProp= erreur* lSettings.kp
					#precommande (moyenne des directions des bords)
					#lPrec = 0.5*lRightAngle+0.5*lLeftAngle
					if lRightAngle>0 and lLeftAngle>0:					
						lPrec = 1.0*min(lRightAngle,lLeftAngle)
					elif lRightAngle<0 and lLeftAngle<0:					
						lPrec = 1.0*max(lRightAngle,lLeftAngle)
					else:
						lPrec = 0.0
					
					# synthese of all terms
					lLastAngle = lPrec + lProp
					
					lLastAngle = max(-30,min(30,lLastAngle))
				elif lLeftDist == 0 and lRightDist != 0:
					lLastAngle = -30
				elif lLeftDist != 0 and lRightDist == 0:
					lLastAngle = 30
					
		lDynamixel.setWheelAngle(lLastAngle)
		# Message publication
		sRosPublisher.publish(emaginarium.msg.CommandNosewheel(lLastAngle,False,lProp,lPrec,lRightAngle,lLeftAngle))
		sRosSuscriberSpeedTarget.publish(emaginarium.msg.SpeedTarget(lLastSpeedTarget))
		
	# Message publication en cas de fermeture de la node on envoie un status
	sRosPublisher.publish(emaginarium.msg.CommandNosewheel(lLastAngle,True,lProp,lPrec,lRightAngle,lLeftAngle))

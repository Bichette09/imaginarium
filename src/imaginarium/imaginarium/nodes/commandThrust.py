#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import pigpio
import time
import imaginarium_core.msg
from settings_store import settings_store_client
import os

class CommandThrust(object):
	def __init__(self,pinA,pinB,pinC,pinD,settings):
		self.speed = 0
		self.thrust = [0,0,0,0]# Thrust table for all the engines
		self.mthrust = 0 # Command Thrust
		self.mGoalSpeed = 0 # Goal Speed
		self.__mIsEnable = False # Engine are disabled
		self.__mGpio = pigpio.pi()
		self.pinA=pinA
		self.pinB=pinB
		self.pinC=pinC
		self.pinD=pinD
		self.sRosPublisher = rospy.Publisher('imaginarium_core/DiagThrust', imaginarium_core.msg.DiagThrust,queue_size=5) 	
		self.settings = settings # Goal Speed propotional gain
		self.updateThrust() # mandatory init engines
		time.sleep(5)
		print('Powertrain is ready')

		
 	def updatexbox (self,param):
		if '|B|' in param.data:
			self.__mIsEnable = not self.__mIsEnable
			rospy.logwarn('Powertrain isEnable: %s'%(self.__mIsEnable))
		
		if '|A|' in param.data:
			if self.mGoalSpeed == 0:
				self.mGoalSpeed = 1
			else:
				self.mGoalSpeed = 0
		self.updateThrust()
				
				
 	def updateSpeed(self,param):
		self.speed = param.speed
		self.updateThrust()
		
	def updateThrust(self):
		# limit goal speed
		self.mGoalSpeed = max(min(-5.,self.mGoalSpeed),5.)
		lError = self.speed - self.mGoalSpeed
		
		self.mThrust = lError * self.settings.kp
		self.mThrust = max(-1.,min(self.mThrust,1.))

		if self.__mIsEnable:
			if self.mThrust >= 0:
				self.thrust[0] = 1050+950*self.mThrust
				self.thrust[1] = 1050
				self.thrust[2] = 1050
				self.thrust[3] = 1050
			else:
				self.thrust[0] = 1050
				self.thrust[1] = 1050-950*self.mThrust
				self.thrust[2] = 1050-950*self.mThrust
				self.thrust[3] = 1050-950*self.mThrust
		else:
			self.thrust[0] = 1000
			self.thrust[1] = 1000
			self.thrust[2] = 1000
			self.thrust[3] = 1000

		# limitation
		for i in range(4):
			if self.thrust[i]<1000:
				self.thrust[i]=1000
			if self.thrust[i]>2000:
				self.thrust[i]=2000	  
	
		# mapping 1->26; 2->19; 3->13 and 4->6
		self.__mGpio.set_servo_pulsewidth(self.pinA, self.thrust[0])
		self.__mGpio.set_servo_pulsewidth(self.pinB, self.thrust[1])
		self.__mGpio.set_servo_pulsewidth(self.pinC, self.thrust[2])
		self.__mGpio.set_servo_pulsewidth(self.pinD, self.thrust[3])

		# Diagnostic message publishing
		msg = imaginarium_core.msg.DiagThrust()
		msg.commandedThrust = self.mthrust
		msg.goalSpeed = self.mGoalSpeed
		msg.thrust = self.thrust
		self.sRosPublisher.publish(msg)

class CommandThrustSettings(settings_store_client.SettingsBase):

	def __init__(self):
		settings_store_client.SettingsBase.__init__(self)
		self.kp=5
		self.registerAttributes([
			('kp','commandThrust/Kp')
			])
		
if __name__ == "__main__":	
	os.getcwd()
	rospy.init_node('commandThrust')

	#creation instance settings pour les parametres modifiables
	lSettings = CommandThrustSettings()

	lCommandThrust = CommandThrust(rospy.get_param('/commandThrust/pinA'),rospy.get_param('/commandThrust/pinB'),rospy.get_param('/commandThrust/pinC'),rospy.get_param('/commandThrust/pinD'),lSettings)
	
	sRosSuscriberSpeed = rospy.Subscriber('imaginarium_core/Speed', imaginarium_core.msg.Speed,lCommandThrust.updateSpeed)
	sRosSuscriberSpeed = rospy.Subscriber('GamePadButtons', imaginarium_core.msg.GamePadButtons,lCommandThrust.updatexbox)
	rospy.spin() # Attente de la mise à jour de la valeur de Speed pour executer updateSpeed
	
		
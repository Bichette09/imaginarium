#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import pigpio
import time
import emaginarium.msg
import emaginarium_common.msg
from settings_store import settings_store_client
import os


class CommandThrust(object):
	def __init__(self,pinA,pinB,pinC,pinD,settings):
		self.speed = 0
		self.thrust = [0,0,0,0]# Thrust table for all the engines
		self.mthrust = 0 # Command Thrust
		self.mGoalSpeed = 0. # Goal Speed
		self.mGpio = pigpio.pi()
		self.pinA=pinA
		self.pinB=pinB
		self.pinC=pinC
		self.pinD=pinD
		self.sRosPublisher = rospy.Publisher('emaginarium/DiagThrust', emaginarium.msg.DiagThrust,queue_size=5) 	
		self.settings = settings # Goal Speed propotional gain
		self.mStateDeclarator = settings_store_client.StateDeclarator()
		self.mPowerWatchdog = settings_store_client.PowerWatchdog(self.mStateDeclarator)
		lSettings = CommandThrustSettings()
		self.mStateDeclarator.setState("actuator/enable","disable")
		self.mStateDeclarator.setState("init/actuator",True)
		try:
			self.updateThrust() # mandatory init engines
		except:
			rospy.logerr('Fail to initialize engines')
			self.mGpio = None
		time.sleep(5)
		print('Powertrain is ready')
			
		self.updateThrust()
				
				
 	def updateSpeed(self,param):
		self.speed = param.speed
		self.updateThrust()
	
	def updateSpeedTarget(self, param):
		self.mGoalSpeed = min(max(-5.,param.speedtarget),5.)
		if self.mGoalSpeed != param.speedtarget:
			rospy.logerr('Goal speed out of range %f' % (param.speedtarget))
		self.updateThrust();
		
	def updateThrust(self):
		# limit goal speed
		
		self.mGoalSpeed = min(max(-5.,self.mGoalSpeed),5.)
		lError = self.speed - self.mGoalSpeed
		
		self.mThrust = lError * self.settings.kTh
		self.mThrust = max(-1.,min(self.mThrust,1.))

		if self.mPowerWatchdog.isPowerEnable():
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
		if self.mGpio is not None:
			self.mGpio.set_servo_pulsewidth(self.pinA, self.thrust[0])
			self.mGpio.set_servo_pulsewidth(self.pinB, self.thrust[1])
			self.mGpio.set_servo_pulsewidth(self.pinC, self.thrust[2])
			self.mGpio.set_servo_pulsewidth(self.pinD, self.thrust[3])
		# Diagnostic message publishing
		msg = emaginarium.msg.DiagThrust()
		msg.commandedThrust = self.mthrust
		msg.goalSpeed = self.mGoalSpeed
		msg.thrust = self.thrust
		self.sRosPublisher.publish(msg)

class CommandThrustSettings(settings_store_client.SettingsBase):

	def __init__(self):
		settings_store_client.SettingsBase.__init__(self)
		self.kTh=5
		self.registerAttributes([
			('kTh','commandThrust/kTh')
			])
		
if __name__ == "__main__":	
	os.getcwd()
	rospy.init_node('commandThrust')

	#creation instance settings pour les parametres modifiables
	lSettings = CommandThrustSettings()
	
	lCommandThrust = CommandThrust(rospy.get_param('/commandThrust/pinA'),rospy.get_param('/commandThrust/pinB'),rospy.get_param('/commandThrust/pinC'),rospy.get_param('/commandThrust/pinD'),lSettings)
	
	sRosSuscriberSpeed = rospy.Subscriber('emaginarium/Speed', emaginarium.msg.Speed,lCommandThrust.updateSpeed)
	sRosSuscriberSpeed = rospy.Subscriber('emaginarium/SpeedTarget', emaginarium.msg.SpeedTarget,lCommandThrust.updateSpeedTarget)
	rospy.spin() # Attente de la mise à jour de la valeur de Speed pour executer updateSpeed
	
		

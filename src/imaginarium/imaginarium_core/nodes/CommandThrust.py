#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import pigpio
import time
import imaginarium_core.msg

class CommandThrust(object):
	def __init__(self):
		self.speed = 0
		self.thrust = [0,0,0,0]# Thrust table for all the engines
		self.mthrust = 0 # Command Thrust
		self.mGoalSpeed = 0 # Goal Speed
		self.__mIsEnable = False # Engine are disabled
		self.__set_thrust([0,0,0,0],False) # mandatory init engines
		self.__mKp = 5 # Goal Speed propotional gain
		time.sleep(5)
		print('Powertrain is ready')	
		
 	def updatexbox (self,param):
		if 'B' in param.data:
			if self.__mIsEnable == True:
				self.__mIsEnable == False
			else
				self.__mIsEnable == True
		
		if 'A' in param.data:
			if self.mGoalspeed == 0:
				self.mGoalSpeed = 1
			else
				self.mGoalspeed = 0:
				
				
 	def updateSpeed(self,param):
		self.speed = param.speed
		# limit goal speed
		self.mGoalSpeed = max(min(-5.,self.mGoalSpeed),5.)
		lError = self.speed - self.mGoalSpeed
		
		self.mThrust = lError * self.__mKp
		self.mThrust = max(-1.,min(self.mThrust,1.))
		setThrust(self, self.mThrust):
		
	def setThrust(self, pThrust):
		listThrust = [0,0,0,0]
		if self.__mIsEnable:
			if pThrust >= 0:
				listThrust[0] = 1050+950*pThrust
				listThrust[1] = 1050
				listThrust[2] = 1050
				listThrust[3] = 1050
			else:
				listThrust[0] = 1050
				listThrust[1] = 1050-950*pThrust
				listThrust[2] = 1050-950*pThrust
				listThrust[3] = 1050-950*pThrust
		# limitation
		if len(thrust)==4:
			for i in range(4):
				if thrust[i]<1000:
					thrust[i]=1000
				if thrust[i]>2000:
					thrust[i]=2000	  
	
		# mapping 1->26; 2->19; 3->13 and 4->6
		self.__mGpio.set_servo_pulsewidth(26, thrust[0])
		self.__mGpio.set_servo_pulsewidth(19, thrust[1])
		self.__mGpio.set_servo_pulsewidth(13, thrust[2])
		self.__mGpio.set_servo_pulsewidth( 6, thrust[3])
		
if __name__ == "__main__":	
	os.getcwd()
	rospy.init_node('commandThrust')

	
	sRosPublisher = rospy.Publisher('imaginarium_core/DiagThrust', imaginarium_core.msg.DiagThrust, ) 
	sRosSuscriberSpeed = rospy.Subscriber('imaginarium_core/Speed', imaginarium_core.msg.Speed,lCommandThrust.updateSpeed)
	sRosSuscriberSpeed = rospy.Subscriber('imaginarium_core/xbox', imaginarium_core.msg.gamepadbuttons,lCommandThrust.updatexbox)
	rospy.spin() # Attente de la mise à jour de la valeur de Speed pour executer updateSpeed
	
		

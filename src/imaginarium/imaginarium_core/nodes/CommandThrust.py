#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import pigpio
import time
  
  def __init__(self, pSensorReader, pServoAdr):
		self.__set_thrust([0,0,0,0],False) # mandatory init engines
		time.sleep(5)
		print('Powertrain is ready')	
  
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
		self.__set_thrust(listThrust,False)
    
    	def __set_thrust(self,thrust,mockup):
		if self.__mGpio is None :
			return
		# limitation
		if len(thrust)==4:
			for i in range(4):
				if thrust[i]<1000:
					thrust[i]=1000
				if thrust[i]>2000:
					thrust[i]=2000	  
		if mockup == True:
			self.__mGpio.set_servo_pulsewidth(26, 0)
			self.__mGpio.set_servo_pulsewidth(19, 0)
			self.__mGpio.set_servo_pulsewidth(13, 0)
			self.__mGpio.set_servo_pulsewidth( 6, 0)
		else:
			# mapping 1->26; 2->19; 3->13 and 4->6
			self.__mGpio.set_servo_pulsewidth(26, thrust[0])
			self.__mGpio.set_servo_pulsewidth(19, thrust[1])
			self.__mGpio.set_servo_pulsewidth(13, thrust[2])
			self.__mGpio.set_servo_pulsewidth( 6, thrust[3])

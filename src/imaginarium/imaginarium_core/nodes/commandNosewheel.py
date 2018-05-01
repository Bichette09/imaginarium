#!/usr/bin/env python
# coding: utf-8

import imaginarium_core.msg
import rospy
import os
import serial
import math
import pypot.dynamixel
import pigpio

class CommandNosewheel(object):
	
	def __init__(self,serialPort,gainKp):
		self.speed = 0
		self.ultrasonDist = [0]*10
		# retrait du capteur arriere (cm)
		self.rd = 1.9
		self.ld = 1.6
		# distance entre le capteur arriere et avant (cm)		
		self.rL = 37.5
		self.lL = 37.5
		#Servomoteur init
		self.__dxl_io = pypot.dynamixel.DxlIO(serialPort)
		if self.__dxl_io is None:
			print('Servo is disabled, fail to create DxlIO object')
		self.__mGpio = pigpio.pi()
		if self.__mGpio is None or not self.__mGpio.connected:
			print('Powertrain is disabled, fail to create GPIO object')
			self.__mGpio = None
		time.sleep(5)
		print('Powertrain is ready')
		#Gains loi
		self.kp = gainKp
	
 	def updateSpeed(self,param):
		self.speed = param.speed
	
	def updateUltrason(self,param):
		self.ultrasonDist = param.distance
	
	def updateAntenna(self,param):
		self.antennaStatus = param.status
		
	def setWheelAngle(self,angle):
		if self.__dxl_io is not None:
			self.__dxl_io.set_goal_position({1:angle})

if __name__ == "__main__":
	
	os.getcwd()
	rospy.init_node('commandNosewheel')

	
	sRosPublisher = rospy.Publisher('imaginarium_core/CommandNosewheel', imaginarium_core.msg.CommandNosewheel, queue_size=5)
	lCommandNosewheel = CommandNosewheel(rospy.get_param('/commandNosewheel/serialPort'),rospy.get_param('/commandNosewheel/Kp'))
	sRosSuscriberUltra = rospy.Subscriber('imaginarium_core/Ultrasound', imaginarium_core.msg.Ultrasound,lCommandNosewheel.updateUltrason)
	sRosSuscriberSpeed = rospy.Subscriber('imaginarium_core/Speed', imaginarium_core.msg.Speed,lCommandNosewheel.updateSpeed)
	
	#init
	cptDoor = 0
	lLastAngle = 0.
	lFrontDist = 0.
	lRightDist = 0.
	lLeftDist = 0.
	lTimeInt=time.time()
	ierreur=0.
	
	while not rospy.core.is_shutdown():
		
		# we want 1kHz reresh rate in practice we will get 700Hz
		time.sleep(0.001)
		

		if lCommandNosewheel.ultrasonDist is not None:
			lRightDist = 0.
			for i in range(0,4):		
				if	lCommandNosewheel.ultrasonDist[i] > 0:
					if lRightDist > 0:
						lRightDist = min(lRightDist,lCommandNosewheel.ultrasonDist[i])
					else:
						lRightDist = lCommandNosewheel.ultrasonDist[i]
			lLeftDist = 0.
			for i in range(6,10):		
				if	lCommandNosewheel.ultrasonDist[i] > 0:
					if lLeftDist > 0:
						lLeftDist = min(lLeftDist,lCommandNosewheel.ultrasonDist[i])
					else:
						lLeftDist = lCommandNosewheel.ultrasonDist[i]
				
			lFrontDist = 0.
			for i in range(3,7):		
				if	lCommandNosewheel.ultrasonDist[i] > 0:
					if lFrontDist > 0:
						lFrontDist = min(lFrontDist,lCommandNosewheel.ultrasonDist[i])
					else:
						lFrontDist = lCommandNosewheel.ultrasonDist[i]

			# angle between robot and runway
			lRightAngle = 0.
			if lCommandNosewheel.ultrasonDist[1]>0. and lCommandNosewheel.ultrasonDist[0]>0.:
					lRightAngle = -1.5-math.atan((import math[1]-(lCommandNosewheel.ultrasonDist[0]+lCommandNosewheel.rd))/lCommandNosewheel.rL)*180/math.pi
				lLeftAngle = 0.
				if lCommandNosewheel.ultrasonDist[8]>0. and lCommandNosewheel.ultrasonDist[9]>0.:
					lLeftAngle =  math.atan((lCommandNosewheel.ultrasonDist[8]-(lCommandNosewheel.ultrasonDist[9]+lCommandNosewheel.ld))/lCommandNosewheel.lL)*180/math.pi


				#print "rightAngle:"+str(lRightAngle)+"\t leftAngle:"+str(lLeftAngle)
				#print "Dist8:"+str(lCommandNosewheel.ultrasonDist[8])+"\t Dist9:"+str(lCommandNosewheel.ultrasonDist[9])
				#print "Dist1:"+str(lCommandNosewheel.ultrasonDist[1])+"\t Dist0:"+str(lCommandNosewheel.ultrasonDist[0])

				
				# compute angle			
				lLastAngle = 0.
				if lLeftDist != 0 and lRightDist != 0:
				#Angle control law
					erreur= (lRightDist-lLeftDist)
					#Proportional term
					lProp= erreur* lCommandNosewheel.kp
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
					
		lCommandNosewheel.setWheelAngle(lLastAngle)
		# Message publication
		sRosPublisher.publish(imaginarium_core.msg.CommandNosewheel(lNoseWheelAngle),False)
		
	# Message publication en cas de fermeture de la node on envoie un status
	sRosPublisher.publish(imaginarium_core.msg.CommandNosewheel(lNoseWheelAngle),True)

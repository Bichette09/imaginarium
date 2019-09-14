#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import time
import emobile.msg
import os
import emaginarium_common.msg
import std_msgs
from settings_store import settings_store_client
import time
import serial

# steering table
# for left : 5710
# for middle : 4915
# for right : 4215

# throttle table
# stop : 4915
# mini : 5060
# max : 6600

class ActuatorSettings(settings_store_client.SettingsBase):

	def __init__(self):
		settings_store_client.SettingsBase.__init__(self)
		self.kTh=0.3
		self.mMaxPower = 0.2
		self.registerAttributes([
			('kTh','actuator/kTh','kTh'),
			('mMaxPower','actuator/maxpower',0.,1.,'max commanded power'),
			])

class Actuator(object):
	def __init__(self):
		self.__mStateDeclarator = settings_store_client.StateDeclarator()
		self.__mPowerWatchdog = settings_store_client.PowerWatchdog(self.__mStateDeclarator)
		self.mRosPublisherThrottle = rospy.Publisher('/throttles', std_msgs.msg.Float32, queue_size=1)
		self.mRosPublisherPingLidardDist = rospy.Publisher('emobile/PingLidarDist', std_msgs.msg.Float32, queue_size=1)
		self.mRosPublisherSpeed = rospy.Publisher('/speed', std_msgs.msg.Float32,queue_size = 1)
		self.mRosPublisherOdometry = rospy.Publisher('/odometry', std_msgs.msg.Float32,queue_size = 1)

	
		self.throttles = 0
		self.steering = 0
		self.settings = ActuatorSettings()
		self.mGoalThrottle=0 #-1 pour marche arriere max 0 arret 1 marche avant max
		self.mGoalSteering=0
		self.mLastNonNullGoalThrottle = 0
		
		self.__mSerialPort = None
		
		try:
			self.__mSerialPort = serial.Serial('/dev/ttyTHS1', baudrate=115200,timeout=1)
		except:
			self.__mSerialPort = None
		self.__mStateDeclarator.setState("actuator/enable","disable")
				
		time.sleep(5)
		if self.__mSerialPort is None:
			self.__mStateDeclarator.setState("init/actuator",False)
			rospy.logerr('Fail to open esp32.')
		else:
			self.__mStateDeclarator.setState("init/actuator",True)
			rospy.logwarn('Powertrain is ready')

	def __del__(self):
		if self.__mSerialPort is not None:
			self.__mSerialPort.write('S4915\n')
			self.__mSerialPort.write('T4915\n')
		

	def updateThrottleTarget(self, param):
		self.mGoalThrottle = min(max(-1.5,param.throttle),4.)
		if self.mGoalThrottle != param.throttle:
			rospy.logerr('Goal speed out of range %f' % (param.throttle))
		self.updateThrottle()
	
	def updateThrottle(self):
		self.mGoalThrottle = min(max(-1.5,self.mGoalThrottle),4.)
		
		lGoalThrottle = 0.
		
		# compute and clamp goal throttle
		lGoalThrottle = self.mGoalThrottle * self.settings.kTh
		lGoalThrottle = min(max(- self.settings.mMaxPower,lGoalThrottle),self.settings.mMaxPower)
		
		# translate goal into pwm pulse width
		if abs(lGoalThrottle) > 0.01:
			# MOVE
			if lGoalThrottle > 0.:
				self.throttles = 1550
				self.throttles = self.throttles + (1980 - self.throttles)*lGoalThrottle
			else:
				self.throttles = 1500
				# for now disable backward, it is not working properly
				# self.throttles = self.throttles + (self.throttles - 980)*lGoalThrottle
		else:
			# STOP
			self.throttles = 1500
		
		# if we are about to close the node, or if power is disabled, emit neutral point
		if rospy.is_shutdown() or not self.__mPowerWatchdog.isPowerEnable():
			self.throttles = 1500
		
		# rospy.logwarn(self.throttles)
		self.mRosPublisherThrottle.publish(std_msgs.msg.Float32(self.throttles))

			

	def updateSteeringTarget(self, param):
		self.mGoalSteering = min(max(-1.,param.steering),1.)
		if self.mGoalSteering != param.steering:
			rospy.logerr('Goal steering out of range %f' % (param.steering))
		self.updateSteering()

	def updateSteering(self):
		self.mGoalSteering = min(max(-1.,self.mGoalSteering),1.)
		self.steering= (1.49-0.25*self.mGoalSteering)*1000

	
	def updateSpeed(self,param):
		self.__mLastSpeed = param.speed
		self.updateThrottle()

	def execute(self):
		self.__mSerialPort.write('R\n')
		rospy.sleep(0.01)
		odo = self.__mSerialPort.readline()

		spe = self.__mSerialPort.readline()

		lid = self.__mSerialPort.readline()

		#rospy.logwarn(odo)
		#rospy.logwarn(spe)
		#rospy.logwarn(lid)
		if len(odo)>0:
			self.mRosPublisherOdometry.publish(std_msgs.msg.Float32(float(odo[1:-2])))
		if len(spe)>0:
			self.mRosPublisherSpeed.publish(std_msgs.msg.Float32(float(spe[1:-2])))
		if len(lid)>0:
			self.mRosPublisherPingLidardDist.publish(std_msgs.msg.Float32(float(lid[1:-2])/1000.0))
		rospy.sleep(0.01)
		self.__mSerialPort.write("S"+str(int(self.steering*4915.0/1500.0))+"\n")
		self.__mSerialPort.write("T"+str(int(self.throttles*4915.0/1500.0))+"\n")

		#rospy.logwarn(self.steering*4915.0/1500.0)
		#rospy.logwarn(self.throttles*4915.0/1500.0)

		

	def close(self):
		#self.__mSerialPort.close() :/!\ ATTENTION, LA FERMETURE DU PORT SERIE ENVOIE A FOND
		pass


		
if __name__ == "__main__":	
	os.getcwd()
	rospy.init_node('actuatoresp32')
	lActuator = Actuator()
	sRosSuscriberThrottle = rospy.Subscriber('emobile/CommandThrottle', emobile.msg.CommandThrottle,lActuator.updateThrottleTarget)
	sRosSuscriberSteering = rospy.Subscriber('emobile/CommandSteering', emobile.msg.CommandSteering,lActuator.updateSteeringTarget)

	lRate = rospy.Rate(42)
	while not rospy.is_shutdown():
		lActuator.execute()
		lRate.sleep()
	lActuator.updateThrottle()
	lActuator.close()

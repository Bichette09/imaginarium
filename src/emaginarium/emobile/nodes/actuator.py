#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import pigpio
import time
import emobile.msg
import os
import emaginarium_common.msg
import std_msgs
from settings_store import settings_store_client
import time


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
	def __init__(self,pinT,pinS):
		self.__mStateDeclarator = settings_store_client.StateDeclarator()
		self.__mPowerWatchdog = settings_store_client.PowerWatchdog(self.__mStateDeclarator);
		self.mRosPublisherThrottle = rospy.Publisher('/throttles', std_msgs.msg.Float32, queue_size=1)
	
		self.throttles = 0
		self.steering = 0
		self.settings = ActuatorSettings()
		self.mGoalThrottle=0 #-1 pour marche arriere max 0 arret 1 marche avant max
		self.mGoalSteering=0
		self.mLastNonNullGoalThrottle = 0
		
		self.__mGpio = None
		
		try:
			self.__mGpio = pigpio.pi()
		except:
			self.__mGpio = None
		self.pinT=pinT
		self.pinS=pinS
		self.__mStateDeclarator.setState("actuator/enable","disable")
				
		time.sleep(5)
		if self.__mGpio is None:
			self.__mStateDeclarator.setState("init/actuator",False)
			rospy.logerr('Fail to open gpio, actuator will not work properly')
		else:
			self.__mStateDeclarator.setState("init/actuator",True)
			rospy.logwarn('Powertrain is ready')

	def __del__(self):
		if self.__mGpio is not None:
			self.__mGpio.set_servo_pulsewidth(self.pinT, 1520)
		

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
		if self.__mGpio is not None:
			self.mRosPublisherThrottle.publish(std_msgs.msg.Float32(self.throttles))
			self.__mGpio.set_servo_pulsewidth(self.pinT, self.throttles)

	def updateSteeringTarget(self, param):
		self.mGoalSteering = min(max(-1.,param.steering),1.)
		if self.mGoalSteering != param.steering:
			rospy.logerr('Goal steering out of range %f' % (param.steering))
		self.updateSteering()

	def updateSteering(self):
		self.mGoalSteering = min(max(-1.,self.mGoalSteering),1.)
		self.steering= (1.49-0.25*self.mGoalSteering)*1000
		if self.__mGpio is not None:
			self.__mGpio.set_servo_pulsewidth(self.pinS, self.steering)
	
	def updateSpeed(self,param):
		self.__mLastSpeed = param.speed
		self.updateThrottle()
		
if __name__ == "__main__":	
	os.getcwd()
	rospy.init_node('actuator')

	lActuator = Actuator(rospy.get_param('/actuator/pinT'),rospy.get_param('/actuator/pinS'))
	sRosSuscriberThrottle = rospy.Subscriber('emobile/CommandThrottle', emobile.msg.CommandThrottle,lActuator.updateThrottleTarget)
	sRosSuscriberSteering = rospy.Subscriber('emobile/CommandSteering', emobile.msg.CommandSteering,lActuator.updateSteeringTarget)
	rospy.spin()
	lActuator.updateThrottle()

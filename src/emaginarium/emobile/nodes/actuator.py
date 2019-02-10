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
		self.mMaxRunTimer = 48
		self.mMinTimeBetweenFinishLine = 36
		self.forwardStopThrottle = 1520
		self.backwardStopThrottle = 1520
		self.registerAttributes([
			('kTh','actuator/kTh','kTh'),
			('forwardStopThrottle','actuator/fwdStopThrottle','command applied to stop emobile when it was moving in forward direction default 1520'),
			('backwardStopThrottle','actuator/bwdStopThrottle','command applied to stop emobile when it was moving in backward direction default 1520'),
			('mMaxRunTimer','actuator/maxruntime','max run time after start'),
			('mMinTimeBetweenFinishLine','actuator/mintimebetweenfinishline','min duration between finish detection'),
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
		self.mBrakeTimer = None
		
		self.mFinishLineTimer = None
		self.mRunTimer = None
		
		self.mIgnoreCommand = False
		self.mGotLight = False
		
		self.__mGpio = None
		self.__mLastSpeed = 0.
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
		self.mGoalThrottle = min(max(-1.,param.throttle),1.)
		if self.mGoalThrottle != param.throttle:
			rospy.logerr('Goal speed out of range %f' % (param.throttle))
		self.updateThrottle()
	
	def onimgdetection(self, param):
	
		if param.data == 'finish_line_detected':
			lNow = time.time()
			if self.mFinishLineTimer is None:
				self.mFinishLineTimer = lNow
			elif (lNow - self.mFinishLineTimer) > self.settings.mMinTimeBetweenFinishLine:
				#self.__mPowerWatchdog.setPowerEnableMsg(False,'Finish line')
				self.mFinishLineTimer=None
				#self.mIgnoreCommand = True
				rospy.logwarn('finish line')
		elif param.data == 'start_light_sequence_detected':
			self.mGotLight = True
			rospy.logwarn('got light')
	
	def updateThrottle(self):
		self.mGoalThrottle = min(max(-1.,self.mGoalThrottle),1.)
		
		lGoalThrottle = 0.
		
		lNow = time.time()
		
		if self.__mPowerWatchdog.isPowerEnable() and not self.mIgnoreCommand and self.mGotLight:
			if self.mRunTimer is None:
				self.mFinishLineTimer = None
				self.mRunTimer = lNow
			lGoalThrottle = self.mGoalThrottle * self.settings.kTh
		elif not self.__mPowerWatchdog.isPowerEnable():
			self.mIgnoreCommand = False
			self.mGotLight = False
			
		if self.mRunTimer is not None:
			if (lNow - self.mRunTimer) > self.settings.mMaxRunTimer:
				#self.__mPowerWatchdog.setPowerEnableMsg(False,'time out')
				self.mRunTimer=None
				self.mIgnoreCommand = True
				rospy.logwarn('time out')
			
		if abs(lGoalThrottle) > 0.01:
			# MOVE
			self.mLastNonNullGoalThrottle = lGoalThrottle
			self.mBrakeTimer = time.time()
			
			if lGoalThrottle > 0.:
				self.throttles = 1540
				self.throttles = self.throttles + (1980 - self.throttles)*lGoalThrottle
			else:
				self.throttles = 1500
				self.throttles = self.throttles + (980-self.throttles)*lGoalThrottle
		else:
			# STOP
			self.throttles = 1520
			
			if (self.mBrakeTimer is not None) and (time.time() - self.mBrakeTimer) < 2.5 and self.mLastNonNullGoalThrottle > 0.:
				#rospy.logwarn('On freine')
				if self.__mLastSpeed > 0.3:
					# we need to brake if we are moving faster than 0.3 m/s
					if self.mLastNonNullGoalThrottle > 0.01:
						self.throttles = self.settings.forwardStopThrottle
					elif self.mLastNonNullGoalThrottle < 0.01:
						self.throttles = self.settings.backwardStopThrottle
				else:
					self.mBrakeTimer = None
			else:
				self.mBrakeTimer = None
				
				
		# rospy.logwarn(self.throttles)
		if self.__mGpio is not None:
			if rospy.is_shutdown():
				self.__mGpio.set_servo_pulsewidth(self.pinT, 1520)
			else:
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
		
if __name__ == "__main__":	
	os.getcwd()
	rospy.init_node('actuator')

	lActuator = Actuator(rospy.get_param('/actuator/pinT'),rospy.get_param('/actuator/pinS'))
	sRosSuscriberThrottle = rospy.Subscriber('emobile/CommandThrottle', emobile.msg.CommandThrottle,lActuator.updateThrottleTarget)
	sRosSuscriberSteering = rospy.Subscriber('emobile/CommandSteering', emobile.msg.CommandSteering,lActuator.updateSteeringTarget)
	sRosSuscriberImg = rospy.Subscriber('/light_and_line_detector/event', std_msgs.msg.String,lActuator.onimgdetection)
	sRosSuscriberSpeed = rospy.Subscriber('/speed', emobile.msg.Speed,lActuator.updateSpeed)
	rospy.spin()
	lActuator.updateThrottle()

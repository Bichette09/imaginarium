#!/usr/bin/env python
# coding: utf-8

import emaginarium.msg
import emaginariumcommonlib
import rospy
import os
import smbus
import serial
import numpy as np
import math

class Ultrasound(object):

		def __init__(self,pSerialPort):
			self.__mSerialPort = None
			self.__mPlotError = True
			self.__mExpectedResLength = 10
			try:
				self.__mSerialPort = serial.Serial(pSerialPort, baudrate=2000000,timeout=1) #Arduino
				self.__mSerialPort.readline() #on purge la premiere trame
			except:
				rospy.logerr('Fail to open serial port with arduino')
			
			
		def readDistance(self):
			lMeasures = []
			if self.__mSerialPort is None:
				return [0]*self.__mExpectedResLength
			try:
				a=self.__mSerialPort.readline()
				b=a[1:-3]
				c=b.split(b",")
				for i in reversed(c):
					lVal = float(i)
					if lVal is None:
						raise Exception('invalid val ' + str(a))
					lMeasures.append(lVal*10.) 
				if len(lMeasures) != 10:
					raise Exception()
				self.__mPlotError = True
			except Exception as e:
				lMeasures = [0]*self.__mExpectedResLength
				if self.__mPlotError:
					rospy.logerr('Fail to read data from arduino ' + str(e))
					self.__mPlotError = False
			
			return lMeasures

# this class is used 
class Distance2PointConverter(object):

	def __init__(self):
		self.mSensorPositions = []
		self.mDirVector = []
		# for i in range(0,10):
			# lAngle = float(i) * np.pi / (10 - 1)
			# lPos = np.array([16 * math.cos(lAngle) , 16 * math.sin(lAngle)])
			# self.mDirVector.append(lPos / np.linalg.norm(lPos))
			# self.mSensorPositions.append(lPos)
		
		# 0 0°
		self.mSensorPositions.append(np.array([170.,18.]))
		self.mDirVector.append(np.array([1.,0.]));
		
		# 1a 20°
		# self.mSensorPositions.append(np.array([165.,82.]))
		# self.mDirVector.append(np.array([0.939693,0.34202]));
		
		# 1b 0°
		self.mSensorPositions.append(np.array([145.,-361.]))
		self.mDirVector.append(np.array([1.,0.]));
		
		# 2 40°
		self.mSensorPositions.append(np.array([132.,132.]))
		self.mDirVector.append(np.array([0.766044,0.642788]));
		
		# 3 62°
		self.mSensorPositions.append(np.array([86.,174.]))
		self.mDirVector.append(np.array([0.469472,0.882948]));
		
		# 4 80°
		self.mSensorPositions.append(np.array([30.,194.]))
		self.mDirVector.append(np.array([0.173648,0.984808]));
		
		# 5 100°
		self.mSensorPositions.append(np.array([-30.,194.]))
		self.mDirVector.append(np.array([-0.173648,0.984808]));
		
		# 6 123°
		self.mSensorPositions.append(np.array([-92.,169.]))
		self.mDirVector.append(np.array([-0.544639,0.838671]));
		
		# 7 145°
		self.mSensorPositions.append(np.array([-137.,140.]))
		self.mDirVector.append(np.array([-0.819152,0.573576]));
		
		# 8a 160°
		# self.mSensorPositions.append(np.array([162.,87.]))
		# self.mDirVector.append(np.array([-0.939693,0.34202]));
	
		# 8b 180°
		self.mSensorPositions.append(np.array([-148.,-355.]))
		self.mDirVector.append(np.array([-1.,0.]));
		
		# 9 180°
		self.mSensorPositions.append(np.array([-174.,24.]))
		self.mDirVector.append(np.array([-1.,0.]));
	
	def computeR0FromDist(self, pDistances):
		lX = []
		lY = []
		try:
			for i in range(0,10):
				if pDistances[i] == 0:
					lX.append(0.)
					lY.append(0.)
				else:
					lPoint = self.mSensorPositions[i] + self.mDirVector[i] * pDistances[i]
					if lPoint[0] is None or lPoint[1] is None:
						lPoint[0] = 0
						lPoint[1] = 0
					lX.append(lPoint[0])
					lY.append(lPoint[1])
				
			return (lX,lY)
		except:
			rospy.logerr('Invalid distance vector %s'%(pDistances))
			return ([0]*10,[0]*10)

if __name__ == "__main__":
	
	os.getcwd()
	rospy.init_node('ultrasound')

	
	sRosPublisher = rospy.Publisher('emaginarium/Ultrasound', emaginarium.msg.Ultrasound, queue_size=5)
	lUltrasoundReader = Ultrasound(rospy.get_param('/ultrasound/serialPort'))
	lDistToPointConv = Distance2PointConverter()

	lFilter = emaginariumcommonlib.DataStreamFilter('ultrasoundfilter')
	
	while not rospy.core.is_shutdown():
	
		lDistance = lUltrasoundReader.readDistance()
		if lDistance is None:
			continue
		(lX0,lY0) = lDistToPointConv.computeR0FromDist(lDistance)
	
		# Message publication
		sRosPublisher.publish(emaginarium.msg.Ultrasound(lDistance,lX0,lY0))

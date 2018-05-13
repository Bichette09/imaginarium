#!/usr/bin/env python
# coding: utf-8

import imaginarium_core.msg
import rospy
import os
import smbus
import serial
import numpy as np
import math

class Ultrasound(object):

		def __init__(self,pSerialPort):
			self.__mSerialPort = serial.Serial(pSerialPort, baudrate=2000000,timeout=1) #Arduino
			self.__mSerialPort.readline() #on purge la premiere trame
			
		def readDistance(self):
			lMeasures = []
			try:
				a=self.__mSerialPort.readline()
				b=a[1:-3]
				c=b.split(b",")
				for i in reversed(c):
					lVal = float(i)
					if lVal is None:
						raise Exception('invalid val ' + str(a))
					lMeasures.append(float(i)) 
				if len(lMeasures) != 10:
					raise Exception()
			except Exception as e:
				rospy.logerr('Fail to read data from arduino ' + str(e))
				lMeasures = [0]*10
			
			return lMeasures

# this class is used 
class Distance2PointConverter(object):

	def __init__(self):
		self.mSensorPositions = []
		self.mDirVector = []
		for i in range(0,10):
			lAngle = float(i) * np.pi / (10 - 1)
			lPos = np.array([16 * math.cos(lAngle) , 16 * math.sin(lAngle)])
			self.mDirVector.append(lPos / np.linalg.norm(lPos))
			self.mSensorPositions.append(lPos)
		
	
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

	
	sRosPublisher = rospy.Publisher('imaginarium_core/Ultrasound', imaginarium_core.msg.Ultrasound, queue_size=5)
	lUltrasoundReader = Ultrasound(rospy.get_param('/ultrasound/serialPort'))
	lDistToPointConv = Distance2PointConverter()

	while not rospy.core.is_shutdown():
	
		lDistance = lUltrasoundReader.readDistance()
		if lDistance is None:
			continue
		(lX0,lY0) = lDistToPointConv.computeR0FromDist(lDistance)
	
		# Message publication
		sRosPublisher.publish(imaginarium_core.msg.Ultrasound(lDistance,lX0,lY0))

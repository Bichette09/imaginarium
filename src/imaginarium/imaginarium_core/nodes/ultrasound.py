#!/usr/bin/env python
# coding: utf-8

import imaginarium_core.msg
import rospy
import os
import smbus
import serial

class Ultrasound(object):

		def __init__(self,pSerialPort):
			self.__mSerialPort = serial.Serial(pSerialPort, baudrate=2000000,timeout=10) #Arduino
			self.__mSerialPort.readline() #on purge la premiere trame
			
		def readDistance(self):
			lMeasures = []
			try:
				a=self.__mSerialPort.readline()
				b=a[1:-3]
				c=b.split(b",")
				for i in reversed(c):
					lMeasures.append(float(i)) 
				
			except Exception as e:
				print('read error ' + str(e))
				lMeasures = []
			
			return lMeasures

# this class is used 
class Distance2PointConverter(object):

	def __init__(self, pConfig):
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
		for i in range(0,10):
			if pDistances[i] == 0:
				lX.append(None)
				lY.append(None)
			else:
				lPoint = self.mSensorPositions[i] + self.mDirVector[i] * pDistances[i]
				lX.append(lPoint[0])
				lY.append(lPoint[1])
			
		return (lX,lY)

if __name__ == "__main__":
	
	os.getcwd()
	rospy.init_node('ultrasound')

	
	sRosPublisher = rospy.Publisher('imaginarium_core/Ultrasound', imaginarium_core.msg.Ultrasound, queue_size=5)
	lUltrasoundReader = Ultrasound(rospy.get_param('/ultrasound/serialPort'))
	lDistToPointConv = Distance2PointConverter()

	while not rospy.core.is_shutdown():
	
		lDistance = lUltrasoundReader.readDistance()
		(lX0,lY0) = lDistToPointConv.(lDistance)
	
		# Message publication
		sRosPublisher.publish(imaginarium_core.msg.Ultrasound(lDistance,lX0,lY0))

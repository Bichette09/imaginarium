#!/usr/bin/env python
# coding: utf-8

import imaginarium_core.msg
import rospy
import os
import smbus
import serial

class SensorReader(object):

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

def computeR0FromDist(distance):
  return none 

if __name__ == "__main__":
	
	os.getcwd()
	rospy.init_node('ultrasound')

	
	sRosPublisher = rospy.Publisher('imaginarium_core/Ultrasound', imaginarium_core.msg.Ultrasound, queue_size=5)
  lUltrasoundReader = Ultrasound(rospy.get_param('/ultrasound/serialPort'))

	while not rospy.core.is_shutdown():
	
		lDistance = lUltrasoundReader.readDistance()
		(lX0,lY0) = computeR0FromDist(lDistance)
	
		# Message publication
sRosPublisher.publish(imaginarium_core.msg.Ultrasound(lDistance,lX0,lY0))

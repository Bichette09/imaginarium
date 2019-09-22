#!/usr/bin/env python
# coding: utf-8

import emaginarium_common.msg
import rospy
import os
import smbus
import time

# define MPU address

class Gyro(object):
	def __init__(self, pIcPort, pMpuAddress):
		self.__mMpuAddress = pMpuAddress
		self.aXOffset = 0
		self.aYOffset = 0
		self.aZOffset = 0
		self.gXOffset = 0
		self.gYOffset = 0
		self.gZOffset = 0
		self.mIsInitialized = False
		try:
			self.__mI2cPort = smbus.SMBus(pIcPort)
			# wake up device and init power management
			self.__mI2cPort.write_byte_data(self.__mMpuAddress,0x6B,0x00)# PWR_MGMT_1
			# who am i
			lWhoAmI = self.__mI2cPort.read_byte_data(self.__mMpuAddress,0x75)
			if lWhoAmI != 0x68:
				rospy.logerr('Fail to init gyro ' + str(lWhoAmI))
				raise Exception()
			# configure gyro
			self.__mI2cPort.write_byte_data(self.__mMpuAddress,0x1B,0x00) # range @ +/- 250°/s
			self.__mAngularSpeedFactor = 250. * 1. / 32768.
					
			# configure accelero
			self.__mI2cPort.write_byte_data(self.__mMpuAddress,0x1C,0x00) # range +/- 2g (we are not in a jet fighter)
			self.__mAccelFactor = 2. * 1. / 32768.

			self.calibrationGyro()
			self.mIsInitialized = True
		except:
			self.__mI2cPort = None
			lError = 'Fail to initialize gyro on port %d @%x' % (pIcPort,self.__mMpuAddress)
			rospy.logerr(lError)
	
	def readAccel(self):
		if self.__mI2cPort is None:
			return [0,0,0]
		lAccelData = self.__mI2cPort.read_i2c_block_data(self.__mMpuAddress,0x3B,6)
		lXAccel = lAccelData[0] * 256 + lAccelData[1]
		if lXAccel > 32767:
			lXAccel -= 65536
		lXAccel *= self.__mAccelFactor
		lYAccel = lAccelData[2] * 256 + lAccelData[3]
		if lYAccel > 32767:
			lYAccel -= 65536
		lYAccel *= self.__mAccelFactor
		lZAccel = lAccelData[4] * 256 + lAccelData[5]
		if lZAccel > 32767:
			lZAccel -= 65536
		lZAccel *= self.__mAccelFactor
		return [lXAccel + self.aXOffset,lYAccel + self.aYOffset,lZAccel + self.aZOffset]
		
	def readAngularSpeed(self):
		if self.__mI2cPort is None:
			return [0,0,0]
		lGyroData = self.__mI2cPort.read_i2c_block_data(self.__mMpuAddress,0x43,6)
		lXGyro = lGyroData[0] * 256 + lGyroData[1]
		if lXGyro > 32767:
			lXGyro -= 65536
		lXGyro *= self.__mAngularSpeedFactor
		lYGyro = lGyroData[2] * 256 + lGyroData[3]
		if lYGyro > 32767:
			lYGyro -= 65536
		lYGyro *= self.__mAngularSpeedFactor
		lZGyro = lGyroData[4] * 256 + lGyroData[5]
		if lZGyro > 32767:
			lZGyro -= 65536
		lZGyro *= self.__mAngularSpeedFactor
		return [lXGyro + self.gXOffset, lYGyro + self.gYOffset, lZGyro + self.gZOffset]
		
	def calibrationGyro(self):
		sumAccel = [0,0,0]
		sumGyro = [0,0,0]
		
		# First 100 measures are discarded
		for i in range(100):
			self.readAccel()
			self.readAngularSpeed()
			time.sleep(0.01)
		
		# Then 100 measures are saved in buffer
		for i in range(100):
			tempAccel = self.readAccel()
			sumAccel[0] = sumAccel[0] + tempAccel[0]
			sumAccel[1] = sumAccel[1] + tempAccel[1]
			sumAccel[2] = sumAccel[2] + tempAccel[2]
			tempGyro = self.readAngularSpeed()
			sumGyro[0] = sumGyro[0] + tempGyro[0]
			sumGyro[1] = sumGyro[1] + tempGyro[1]
			sumGyro[2] = sumGyro[2] + tempGyro[2]
			time.sleep(0.01)
		
		# Compute 100 measures mean
		self.aXOffset = - sumAccel[0] / 100
		self.aYOffset = - sumAccel[1] / 100
		self.aZOffset = 0.981 - (sumAccel[2] / 100)
		self.gXOffset = - sumGyro[0] / 100
		self.gYOffset = - sumGyro[1] / 100
		self.gZOffset = - sumGyro[2] / 100
		
		

if __name__ == "__main__":
	
	os.getcwd()
	rospy.init_node('gyro')

	
	sRosPublisher = rospy.Publisher('emaginarium_common/Gyro', emaginarium_common.msg.Gyro, queue_size=5)
	lGyroReader = Gyro(rospy.get_param('/gyro/i2cPort'),rospy.get_param('/gyro/sMpuAddress'))

	while not rospy.core.is_shutdown() and lGyroReader.mIsInitialized:
	
		lAccel = lGyroReader.readAccel()
		lGyro = lGyroReader.readAngularSpeed()
	
		# Message publication
		sRosPublisher.publish(emaginarium_common.msg.Gyro(lAccel,lGyro))
		time.sleep(0.05)

#!/usr/bin/env python
# coding: utf-8

import gyro.msg
import rospy
import os
import smbus

# define MPU address
sMpuAddress = 0x69
class Gyro(object):
	def __init__(self, pIcPort):
		self.__mI2cPort = smbus.SMBus(pIcPort)
		# wake up device and init power management
		self.__mI2cPort.write_byte_data(sMpuAddress,0x6B,0x00)# PWR_MGMT_1
		
		# who am i
		lWhoAmI = self.__mI2cPort.read_byte_data(sMpuAddress,0x75)
		if lWhoAmI != 0x68:
			raise Exception('Fail to init gyro ' + str(lWhoAmI))

		# configure gyro
		self.__mI2cPort.write_byte_data(sMpuAddress,0x1B,0x00) # range @ +/- 250°/s
		self.__mAngularSpeedFactor = 250. * 1. / 32768.
		
		# configure accelero
		self.__mI2cPort.write_byte_data(sMpuAddress,0x1C,0x00) # range +/- 2g (we are not in a jet fighter)
		self.__mAccelFactor = 2. * 1. / 32768.
	
	def readAccel(self):
		lAccelData = self.__mI2cPort.read_i2c_block_data(sMpuAddress,0x3B,6)
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
		return [lXAccel,lYAccel,lZAccel]
		
	def readAngularSpeed(self):
		lGyroData = self.__mI2cPort.read_i2c_block_data(sMpuAddress,0x43,6)
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
		return [lXGyro,lYGyro,lZGyro]


if __name__ == "__main__":
	
	os.getcwd()
	rospy.init_node('gyro')
	
	if lSettingsFile is None:
		rospy.logfatal('Need /settings_store/settingsfilename parameter, to disable persistent storage give an empty filename')
		raise Exception()
		
	sRosPublisher = rospy.Publisher('gyro/Measures', gyro.msg.measures, queue_size=5)
	lGyroReader = Gyro(rospy.get_param('i2cPort'))

	while not rospy.core.is_shutdown():
	
		lAccel = lGyroReader.readAccel()
		lGyro = lGyroReader.readAngularSpeed()
	
		# Message publication
		sRosPublisher.publish(settings_store.msg.measures(lAccel,lGyro))


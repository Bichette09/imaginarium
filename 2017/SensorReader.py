# coding: utf8
import time
import sys
import json

# this module contains class used to read values from sensors
# to provide them to MapBuilder



# this class read sensor values from a file containing prerecorded measures
class FileReader(object):

	def __init__(self, pFilePath, pColCount):
		self.mFile = open(pFilePath)
		self.mFileIterator = iter(self.mFile)
		self.mColCount = pColCount
		self.mRowCounter = 0
		self.mPreviousTimeStamp = None
	
	def getSensorCount(self):
		return self.mColCount
	
	def __iter__(self):
		return self
	
	def __next__(self):
		self.mRowCounter = self.mRowCounter + 1
		
		# fetch next value
		lRowContent = self.mFileIterator.__next__()
		if lRowContent is None:
			raise StopIteration()
		lResult =([],0.,0.,0.,0.)
		try:
			lJson = json.loads(lRowContent)
			lResult = (lJson[0],lJson[1])
			if self.mPreviousTimeStamp is not None:
				lToWait = lJson[1] - self.mPreviousTimeStamp
				time.sleep(lToWait)
			self.mPreviousTimeStamp = lJson[1]
		
		except:
			print(sys.exc_info())
			print('fail to parse row ' + lRowContent)
			raise StopIteration()
		if len(lResult[0]) != self.mColCount:
			print('invalid column count for row ' + lRowContent)
			raise StopIteration()
			
		return lResult

try:
	import serial
	from rot import RotaryEncoder
	import smbus
	import struct
	import array
	
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
	
	# this class read sensor values sent by the Arduino from serial bus
	class SensorReader(object):

		def __init__(self,pSerialPort):
			self.__mSerialPort = serial.Serial(pSerialPort, baudrate=2000000,timeout=10) #Arduino
			self.__mSerialPort.readline() #on purge la premiere trame

			self.__mTickCounter = 0
			self._rotaryEncoder = RotaryEncoder(24,23)
			self._rotaryEncoder.when_rotated = self.onCountChange
		
			self.__mWheelAngle = 0.
			self.__mPreviousTimeStamp = None
			
		def onCountChange(self, pValue):
			self.__mTickCounter = self.__mTickCounter + pValue

		def setWheelAngle(self, pAngle):
			self.__mWheelAngle = pAngle

		def getSensorCount(self):
			return 10
		
		def iter(self):
			return self.__iter__()

		def next(self):
			return self.__next__() 

		def __iter__(self):
			return self
		
		def __next__(self):
			lMeasures = []
			lDistance = 0
			lSpeed = 0
			try:
				a=self.__mSerialPort.readline()
				b=a[1:-3]
				c=b.split(b",")
				for i in reversed(c):
					lMeasures.append(float(i)) 
				lTickCount = self.__mTickCounter
				self.__mTickCounter = 0
				lDistance = lTickCount * 0.01263
				
			except Exception as e:
				print('read error ' + str(e))
				lMeasures = []
			
			lNewTimeStamp = time.time()
			if self.__mPreviousTimeStamp is not None:
				lSpeed = lDistance / (lNewTimeStamp - self.__mPreviousTimeStamp)
			self.__mPreviousTimeStamp = lNewTimeStamp
			
			return (lMeasures,lNewTimeStamp,lDistance, lSpeed ,self.__mWheelAngle)
except:
	print('Fail to import serial, SensorReader will not be available')


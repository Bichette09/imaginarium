import rospy
import time
import serial

class ArduinoFloatArrayReader(object):

	def __init__(self,pSerialPort,pArrayLength,pSpeed,pName):
		self.__mSerialPort = None
		self.__mPlotError = True
		self.__mName = pName
		self.mExpectedResLength = pArrayLength
		try:
			self.__mSerialPort = serial.Serial(pSerialPort, baudrate=pSpeed,timeout=1) #Arduino
			self.__mSerialPort.readline() #on purge la premiere trame
		except:
			rospy.logerr('Fail to open serial port with ' + pName + ' ' + pSerialPort)
		
		
	def readDistance(self):
		lMeasures = []
		if self.__mSerialPort is None:
			time.sleep(0.01);
			return [0]*self.mExpectedResLength
		try:
			a=self.__mSerialPort.readline()
			b=a[1:-3]
			c=b.split(b",")
			for i in reversed(c):
				lVal = float(i)
				if lVal is None:
					raise Exception('invalid val ' + str(a) + ' ' + self.__mName)
				lMeasures.append(lVal*10.) 
			if len(lMeasures) != self.mExpectedResLength:
				raise Exception()
			self.__mPlotError = True
		except Exception as e:
			time.sleep(0.01);
			lMeasures = [0]*self.mExpectedResLength
			if self.__mPlotError:
				rospy.logerr('Fail to read data from arduino ' + self.__mName + ' ' + str(e))
				self.__mPlotError = False
		
		return lMeasures
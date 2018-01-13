#!/usr/bin/env python
# -*- coding: utf-8 -*-

import traceback
import SensorReader
import InputFilter
import zmq
import base64
import json
import pypot.dynamixel
import pigpio
import time
import sys
import antenne
#from autoThrust import autoThrust

def commande_basic():
	lInputFilters = InputFilter.InputArrayFilter(lConf)

	lStart = time.time()
	lSamples = 0
	for (lSensorInputs,lTimestamp, lMoveLen, lMoveAngle) in lSensorReader:
		if lSensorInputs is None or len(lSensorInputs) != lConf.mSensorCount:
			continue
		lSamples = lSamples +1 
		if lSamples > 50:
			break
		lInputFilters.addSensorMeasures(lSensorInputs,lTimestamp)
		lFilteredSensorInputs = lInputFilters.getFilteredResults()

		lRightDist = 0.
		for i in range(0,3):		
			if	lFilteredSensorInputs[i] > 0:
				if lRightDist > 0:
					lRightDist = min(lRightDist,lFilteredSensorInputs[i])
				else:
					lRightDist = lFilteredSensorInputs[i]
		lLeftDist = 0.
		for i in range(7,10):		
			if	lFilteredSensorInputs[i] > 0:
				if lLeftDist > 0:
					lLeftDist = min(lLeftDist,lFilteredSensorInputs[i])
				else:
					lLeftDist = lFilteredSensorInputs[i]

		lA = 0.
		if lLeftDist != 0 and lRightDist != 0:
			lA = (lRightDist-lLeftDist) * 0.5
			lA = max(-30,min(30,lA))
		elif lLeftDist == 0 and lRightDist != 0:
			lA = -30
		elif lLeftDist != 0 and lRightDist == 0:
			lA = 30
		#print('left ' + str(lLeftDist) + ' right ' + str(lRightDist) + ' ' + str( lA))
		#print(str(lSensorInputs) + ' ' + str(lA) )
		dxl_io.set_goal_position({1:lA})
	lStop = time.time()
	lDelta = lStop - lStart
	print(str(lSamples / lDelta) + 'samples / sec')
	sys.exit(0)


class Command(object):
	def __init__(self, Kp):
		self.mGoalSpeed = 0
		self.__mKp = Kp
		self.mThrust = 0
		
	def onLoop(self, pMap):
		lSpeed = pMap.mTraveledSpeed
		# limit goal speed
		lGoalSpeed = max(min(-5.,self.mGoalSpeed),5.)
		lError = lSpeed - self.mGoalSpeed
		
		self.mThrust = lError * self.__mKp
		self.mThrust = max(-1.,min(self.mThrust,1.))
		
		
class Powertrain(object):
	def __init__(self, pSensorReader, pServoAdr):
		# object used to command angle of the wheel	
		self.__dxl_io = pypot.dynamixel.DxlIO(pServoAdr)
		if self.__dxl_io is None:
			print('Servo is disabled, fail to create DxlIO object')
		self.__mSensorReader = pSensorReader
		self.__mIsEnable = False
		self.__mGpio = pigpio.pi()
		if self.__mGpio is None or not self.__mGpio.connected:
			print('Powertrain is disabled, fail to create GPIO object')
			self.__mGpio = None
		self.__set_thrust([0,0,0,0],False) # mandatory init engines
		time.sleep(5)
		print('Powertrain is ready')

	def setWheelAngle(self,angle):
		if self.__dxl_io is not None:
			self.__dxl_io.set_goal_position({1:angle})
		self.__mSensorReader.setWheelAngle(angle)

	def __set_thrust(self,thrust,mockup):
		if self.__mGpio is None :
			return
		# limitation
		if len(thrust)==4:
			for i in range(4):
				if thrust[i]<1000:
					thrust[i]=1000
				if thrust[i]>2000:
					thrust[i]=2000	  
		if mockup == True:
			self.__mGpio.set_servo_pulsewidth(26, 0)
			self.__mGpio.set_servo_pulsewidth(19, 0)
			self.__mGpio.set_servo_pulsewidth(13, 0)
			self.__mGpio.set_servo_pulsewidth( 6, 0)
		else:
			# mapping 1->26; 2->19; 3->13 and 4->6
			self.__mGpio.set_servo_pulsewidth(26, thrust[0])
			self.__mGpio.set_servo_pulsewidth(19, thrust[1])
			self.__mGpio.set_servo_pulsewidth(13, thrust[2])
			self.__mGpio.set_servo_pulsewidth( 6, thrust[3])

	def isEnable(self):
		return self.__mIsEnable
		
	def setEnable(self, pIsEnable):
		if pIsEnable == self.__mIsEnable:
			return
		self.__mIsEnable = pIsEnable
		
		if self.__mIsEnable:
			self.__dxl_io.enable_torque([1])
			print('Enable Powertrain')
		else:
			self.__dxl_io.disable_torque([1])
			print('Disable Powertrain')
		self.setThrust(0.)
		time.sleep(1)
		
	def setThrust(self, pThrust):
		listThrust = [0,0,0,0]
		if self.__mIsEnable:
			if pThrust >= 0:
				listThrust[0] = 1050+950*pThrust
				listThrust[1] = 1050
				listThrust[2] = 1050
				listThrust[3] = 1050
			else:
				listThrust[0] = 1050
				listThrust[1] = 1050-950*pThrust
				listThrust[2] = 1050-950*pThrust
				listThrust[3] = 1050-950*pThrust
		self.__set_thrust(listThrust,False)
		
if __name__ == '__main__':
	# create conf
	lConf = InputFilter.Configuration()
	lConf.mFilterTimeWindowSec = 0.5
	lConf.mFilterMinimumTimeWindowSec = 0.4
	
	lGyroReader = SensorReader.Gyro(1)
	
	lSensorReader = SensorReader.SensorReader('/dev/ttyACM1')
	lPowerTrain = Powertrain(lSensorReader,'/dev/ttyACM0')

	lAntenna = antenne.Antenne(20,21)
	
	# zmq objects used to retrieve xbox pad informations
	lZmqPadContext = zmq.Context()
	lZmqPadSocket = lZmqPadContext.socket(zmq.REP)
	lZmqPadSocket.bind("tcp://0.0.0.0:8000")

	lZmqPadPoller = zmq.Poller()
	lZmqPadPoller.register(lZmqPadSocket, zmq.POLLIN)
		

	# lAutoThrust = autoThrst(23, 24, 0.01263, 1, 0, 0)
	
	# zmq objects used to send map
	lZmqMapContext = zmq.Context()
	lZmqMapSocket = lZmqMapContext.socket(zmq.PUB)
	lZmqMapSocket.bind("tcp://*:5556")
	
	
	# initialisation
	lMapBuilder = InputFilter.MapBuilder(lSensorReader,lConf)
	lMapBuilder.start()
	lMap = None
	lPreviousMapRevision = 0

	lStart = time.time()
	lLoopCounter = 0
	
	lCommand = Command(Kp = 5)
	
	lIsAutomaticMode = False
	
	lLoopTimerStart = time.time()

	cptDoor = 0
	lLastAngle = 0.
	lFrontDist = 0.
	lRightDist = 0.
	lLeftDist = 0.
	lTimeInt=time.time()
	ierreur=0.
	lDamp=0.
	lInt=0.
	lProp=0.
	try:
		while lMapBuilder.is_alive():
			
			# we want 1kHz reresh rate in practice we will get 700Hz
			time.sleep(0.001)
			
			# lLoopCounter = lLoopCounter + 1
			# lDelta = time.time() - lStart
			# if lDelta > 2:
				# print('Main loop @' + str(lLoopCounter / lDelta) + ' loop/sec')
				# lStart = time.time()
				# lLoopCounter = 0
			
			lAccel = lGyroReader.readAccel()
			lGyro = lGyroReader.readAngularSpeed()
			
			
			#retrieve the map
			lMap = lMapBuilder.getCurrentMap(lMap)
			
			lCommand.onLoop(lMap)
			lPowerTrain.setThrust(lCommand.mThrust)
			
			#convert into json
			if lPreviousMapRevision != lMap.mMapRevision:
				# if lMap.mMapRevision - lPreviousMapRevision > 1:
					# print('we are too slow, we miss a map')
				lPreviousMapRevision = lMap.mMapRevision
				
				lMapJson = lMap.convertToMapViewerJson()
				lMapJson['speedgoal'] = lCommand.mGoalSpeed
				lMapJson['speedmeasured'] = lMap.mTraveledSpeed
				lMapJson['wheelangle'] = lLastAngle
				lMapJson['thrust'] = lCommand.mThrust
				lMapJson['islock'] = lPowerTrain.isEnable()
				lMapJson['isautomatic'] = lIsAutomaticMode
				lMapJson['doorcptr'] = cptDoor
				lMapJson['leftdist'] = lLeftDist
				lMapJson['rightdist'] = lRightDist
				lMapJson['frontdist'] = lFrontDist
				#lMapJson['ax'] = lAccel[0]
				lMapJson['ax'] = lProp
				lMapJson['ay'] = lInt
				lMapJson['wz'] = lDamp
				#lMapJson['ay'] = lAccel[1]
				#lMapJson['wz'] = lGyro[2]
				
				lZmqMapSocket.send(b"mapviewer" + json.dumps(lMapJson).encode('utf-8'))
			lUseXBox = False
			if lUseXBox:
				lPadReply = None
				while True:
					socks = dict(lZmqPadPoller.poll(0))
					if socks:
						if socks.get(lZmqPadSocket) == zmq.POLLIN:
							lPadReply = lZmqPadSocket.recv_json(zmq.NOBLOCK)
						else:
							break
					else:
						break

				if lPadReply is not None and lPadReply.has_key("pad"):
					pad = lPadReply["pad"]
		
					if pad['B']:
						lPowerTrain.setEnable(not lPowerTrain.isEnable())
						lIsAutomaticMode = False
						lCommand.mGoalSpeed = 0.
						cptDoor=0
						print('Hard reset')
						
					if pad['A']:
						lIsAutomaticMode = not lIsAutomaticMode
						cptDoor=0
						print('Automatic mode is ' + str(lIsAutomaticMode))
						time.sleep(0.5)
					
					if not lIsAutomaticMode:
						(lStickX, lStickY) = pad['stickTrim']
						if lStickX != 0.:
							print('STOP')
							lCommand.mGoalSpeed = 0.
							time.sleep(0.5)
						elif lStickY > 0:
							lCommand.mGoalSpeed = lCommand.mGoalSpeed + 0.1
							print('Increase speed ' + str(lCommand.mGoalSpeed))
							time.sleep(0.5)
						elif lStickY < 0:
							lCommand.mGoalSpeed = lCommand.mGoalSpeed - 0.1
							print('Decrease speed ' + str(lCommand.mGoalSpeed))
							time.sleep(0.5)
						
					lZmqPadSocket.send_json({})

			if lMap is not None and lMap.mLastFilteredSensorInput is not None:
				lRightDist = 0.
				for i in range(0,4):		
					if	lMap.mLastFilteredSensorInput[i] > 0:
						if lRightDist > 0:
							lRightDist = min(lRightDist,lMap.mLastFilteredSensorInput[i])
						else:
							lRightDist = lMap.mLastFilteredSensorInput[i]
				lLeftDist = 0.
				for i in range(6,10):		
					if	lMap.mLastFilteredSensorInput[i] > 0:
						if lLeftDist > 0:
							lLeftDist = min(lLeftDist,lMap.mLastFilteredSensorInput[i])
						else:
							lLeftDist = lMap.mLastFilteredSensorInput[i]
				
				lFrontDist = 0.
				for i in range(3,7):		
					if	lMap.mLastFilteredSensorInput[i] > 0:
						if lFrontDist > 0:
							lFrontDist = min(lFrontDist,lMap.mLastFilteredSensorInput[i])
						else:
							lFrontDist = lMap.mLastFilteredSensorInput[i]
				# compute speed
				if lIsAutomaticMode:
					lMaxSpeed = 1.5
					lMinSpeed = 0.5
					lSpeed = 0.
					if lFrontDist <= 0. or lFrontDist > 150.:
						lSpeed = lMaxSpeed
					elif lFrontDist < 40:
						lSpeed = 0.
					elif lFrontDist < 70:
						lSpeed = lMinSpeed
					else:
						lFactor = (lFrontDist - 70) / 80
						lSpeed = lMinSpeed + (lMaxSpeed - lMinSpeed)*lFactor
					lCommand.mGoalSpeed = lSpeed
					print('dist ' + str(lFrontDist) + ' goal ' + str(lSpeed) + ' current ' +str(lMap.mTraveledSpeed))
					

				# compute angle			
				lLastAngle = 0.
				if lLeftDist != 0 and lRightDist != 0:
				#Angle control law
					lKi = 0.25  #Integral gain
					lKp = 0.8  #Proportional gain
					lKd = 0.2  #Damping gain 
					erreur= (lRightDist-lLeftDist)
					#Integral term 
					dt=time.time()-lTimeInt
					lTimeInt = time.time()
					ierreur = ierreur + (erreur *dt)
					lInt= ierreur * lKi
					#Proportional term
					lProp= erreur* lKp
					#Damping term
					lDamp=lGyro[2]* lKd
					#Integral term limitation
					#lInt= max(-30 + lProp,min(30 - lProp, lInt))
					# Control law computation
					lLastAngle = lInt + lProp
					lLastAngle = max(-30,min(30,lLastAngle))
				elif lLeftDist == 0 and lRightDist != 0:
					lLastAngle = -30
				elif lLeftDist != 0 and lRightDist == 0:
					lLastAngle = 30
					
				#if lMap.mTraveledSpeed > 0.8:
					#lLastAngle = max(-15.,min(lLastAngle,15.))

				lPowerTrain.setWheelAngle(lLastAngle)

				# start arrival detection
				if lAntenna.getAndReset():
					print('On a passe la porte')
					cptDoor = cptDoor+1
					if cptDoor >= 2:
						lIsAutomaticMode = False
						lCommand.mGoalSpeed = 0.
						print('on demande l arret')

					
			# sleep
			#lMapBuilder.join(0.05)
	except Exception, err:
		print('Got a General Exception ')
		#traceback.print_tb( sys.last_traceback)
		print(str(err))
		
		lPowerTrain.setThrust(0.)
		time.sleep(2)
		lPowerTrain.setEnable(False)
		time.sleep(0.5)
		raise Exception('Halt due to exception')

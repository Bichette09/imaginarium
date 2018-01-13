#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import numpy as np
import json
import math
import threading
import time
import base64
import copy
import zmq

class Configuration(object):
	def __init__(self):
		# parameters of InputFilter
		
		self.mEnableFilter = False
		# how many time we keept measures in the filter
		self.mFilterTimeWindowSec = 1.5
		# minimum elapsed time between first valid sample and last samples to consider window as valid
		self.mFilterMinimumTimeWindowSec = 1.
		# minimum percentage of valid samples before filter is disabled
		self.mValidSamplesDisableThPercent = 0.4
		# minimum percentage of valid samples to enable filter
		self.mValidSamplesEnableThPercent = 0.6
		
		# parameters of MapBuilder
		self.mMapTimeWindowSec = 2.
		
		# number of sensors
		self.mSensorCount = 10
		

# this class is used to filter an array of input from proximity sensors
class InputArrayFilter(object):
	
	def __init__(self, pConfig):
		self.__mConfig = pConfig
		self.__mFilters = []
		for i in range(0,self.__mConfig.mSensorCount):
			self.__mFilters.append(InputFilter(pConfig))

	def addSensorMeasures(self, pMeasures, pTs):
		if len(pMeasures) != self.__mConfig.mSensorCount:
			print('discard invalid measure array ' + str(pMeasures))
		for i in range(0,self.__mConfig.mSensorCount):
			self.__mFilters[i].addMeasure(pMeasures[i],pTs);

	def getFilteredResults(self):
		lRes = []
		for lFilter in self.__mFilters:
			lRes.append(lFilter.getValue())
		return lRes

# this class is used to filter input of a single proximity sensor
class InputFilter(object):

	def __init__(self, pConfig):
		self.__mPreviousMeasures = []
		self.__mConfig = pConfig
		self.__mIsFilterValid = False

	def addMeasure(self, pMeasure,pTs):
		# remove old measures
		lMinTs = pTs - self.__mConfig.mFilterTimeWindowSec
		while len(self.__mPreviousMeasures) > 0 :
			(lMeasure,lTs) = self.__mPreviousMeasures[0]
			if lTs > lMinTs and lMeasure is not None:
				break;
			self.__mPreviousMeasures.pop(0)
		
		# push new measure
		# if measure is less than 4 cm discard it (due to sensor precision)
		if pMeasure >= 4:
			self.__mPreviousMeasures.append((pMeasure,pTs))
		else:
			self.__mPreviousMeasures.append((None,pTs))

		if self.__mConfig.mEnableFilter:		
			# count how many samples are valid
			lValidCptr = 0
			lValidArray = []
			for (lMeasure,lTs) in self.__mPreviousMeasures:
				if lMeasure is None:
					continue
				lValidArray.append(lMeasure)
				lValidCptr = lValidCptr + 1
			lValidPercent = lValidCptr / float(len(self.__mPreviousMeasures))
			lEffectiveTimeWindowSize = self.__mPreviousMeasures[-1][1] - self.__mPreviousMeasures[0][1]
		
			if self.__mIsFilterValid:
				self.__mIsFilterValid = (lEffectiveTimeWindowSize > self.__mConfig.mFilterMinimumTimeWindowSec) and (lValidPercent > self.__mConfig.mValidSamplesDisableThPercent)
			else:
				self.__mIsFilterValid = (lEffectiveTimeWindowSize > self.__mConfig.mFilterMinimumTimeWindowSec) and (lValidPercent > self.__mConfig.mValidSamplesEnableThPercent)

			if self.__mIsFilterValid:
				self.mValue = np.median(np.array(lValidArray))
				# self.mValue = np.average(np.array(lValidArray))
			else:
				self.mValue = 0.
		else:
			self.mValue = 0
			if self.__mPreviousMeasures[-1][0] is not None:
				self.mValue = self.__mPreviousMeasures[-1][0]
		
	def getValue(self):
		return self.mValue;

# this class is used 
class Distance2PointConverter(object):

	def __init__(self, pConfig):
		self.mConfig = pConfig
		self.mSensorPositions = []
		self.mDirVector = []
		for i in range(0,self.mConfig.mSensorCount):
			lAngle = float(i) * np.pi / (self.mConfig.mSensorCount - 1)
			lPos = np.array([16 * math.cos(lAngle) , 16 * math.sin(lAngle)])
			self.mDirVector.append(lPos / np.linalg.norm(lPos))
			self.mSensorPositions.append(lPos)
	
	def convertDistance2Point(self, pDistances):
		lRes = []
		for i in range(0,self.mConfig.mSensorCount):
			if pDistances[i] == 0:
				lRes.append(None)
			else:
				lRes.append(self.mSensorPositions[i] + self.mDirVector[i] * pDistances[i])
			
		return lRes
		
class MapPoint(object):
	def __init__(self):
		self.mPos = np.array([0.,0.,1.])
		self.mDir = np.array([1.,0.,0.])
		self.mTs = 0
		self.mSensorId = 0
		self.mDist2Sensor = 0.
		self.mInfluenceDist = 0.
		
	def toJson(self):
		return {
			'pt':self.mPos[0:2].tolist(),
			'dir':self.mDir[0:2].tolist(),
			'ts':self.mTs,
			'sensorid':self.mSensorId,
			'dist2sensor':self.mDist2Sensor,
			'influencedist':self.mInfluenceDist
		}

class Map(object):
	def __init__(self, pSensorPositions):
		self.mAllPoints = []
		self.mMapRevision = 0
		self.mTimeStampMin = None
		self.mTimeStampMax = None
		self.mLastSensorInput = None
		self.mLastFilteredSensorInput = None
		self.mSensorLocation = []
		self.mTraveledDistance = 0
		self.mTraveledSpeed = 0
		for lLoc in pSensorPositions:
			self.mSensorLocation.append([lLoc[0],lLoc[1]])
	
	def convertToMapViewerJson(self):
		lJson = {
			'hitpoints' : [],
			'lastinput':self.mLastSensorInput,
			'lastfilteredinput':self.mLastFilteredSensorInput,
			'ts':self.mTimeStampMax,
			'sensorloc':self.mSensorLocation,
			'traveleddistance':self.mTraveledDistance,
			'traveledspeed':self.mTraveledSpeed
			}
		for lPoint in self.mAllPoints:
			lJson['hitpoints'].append(lPoint.toJson())
		return lJson
	
class MapBuilder(threading.Thread):

	def __init__(self, pInputProvider, pConfig):
		threading.Thread.__init__(self)
		if pInputProvider.getSensorCount() != pConfig.mSensorCount:
			raise Exception('sensor count missmatch')
		self.daemon = True
		self.__mDistanceConverter = Distance2PointConverter(pConfig)
		# map used by the thread to do computations
		self.__mThreadMap = Map(self.__mDistanceConverter.mSensorPositions)
		self.__mConfig = pConfig
		self.__mInputProvider = pInputProvider
		self.__mSharedMap = copy.deepcopy(self.__mThreadMap)
		self.__mMutex = threading.Lock()
		self.__mInputFilters = InputArrayFilter(pConfig)
		
	
	def getCurrentMap(self, pPreviousMap):
		lRes = pPreviousMap
		self.__mMutex.acquire()
		if (pPreviousMap is None) or (pPreviousMap.mMapRevision != self.__mSharedMap.mMapRevision):
			lRes = copy.deepcopy(self.__mSharedMap)
		self.__mMutex.release()
		return lRes
		
	def addMeasureToThreadMap(self, pRawInputs, pFilteredInputs, pTs, pMatrixFromPreviousLocation,pDistance,pSpeed):
		lMap = self.__mThreadMap
		
		if pMatrixFromPreviousLocation is None:
			# if matrix is None discard all previous data
			lMap.mAllPoints = []
		else:
			lMinTs = pTs - self.__mConfig.mMapTimeWindowSec
			# discard data that are too old
			while len(lMap.mAllPoints) > 0:
				lTs = lMap.mAllPoints[0].mTs
				if lTs > lMinTs:
					break
				lMap.mAllPoints.pop(0)
			# apply xform to remaining points
			for lPoint in lMap.mAllPoints:
				lPoint.mPos = np.dot(pMatrixFromPreviousLocation,lPoint.mPos).getA1()
				lPoint.mDir = np.dot(pMatrixFromPreviousLocation,lPoint.mDir).getA1()
	
		
		lMap.mLastSensorInput = pRawInputs
		lMap.mLastFilteredSensorInput = pFilteredInputs
		lMap.mTraveledDistance = pDistance
		lMap.mTraveledSpeed = pSpeed
		# retrieve filtered values and convert them into 2D location
		lPoints = self.__mDistanceConverter.convertDistance2Point(pFilteredInputs)
		
		# for now discard all previous entries
		lMap.mMapRevision = lMap.mMapRevision + 1
		lMap.mTimeStampMin = pTs
		lMap.mTimeStampMax = pTs
		for i in range(0,len(lPoints)):
			if lPoints[i] is None:
				continue
			# based on distance to sensor and sensor influence cone (15°) we compute theorical segment on which the measure should be
			# use a simple pythagore
			lSegmentLength = 2*math.tan(math.radians(7.5)) * lMap.mLastFilteredSensorInput[i]
			lNewPoint = MapPoint()
			lNewPoint.mPos[0:2] = lPoints[i]
			lNewPoint.mDir[0:2] = self.__mDistanceConverter.mDirVector[i]
			lNewPoint.mTs = pTs
			lNewPoint.mSensorId = i
			lNewPoint.mDist2Sensor = lMap.mLastFilteredSensorInput[i]
			lNewPoint.mInfluenceDist = lSegmentLength
			lMap.mAllPoints.append(lNewPoint)
		
		for lPoint in lMap.mAllPoints:
			lMap.mTimeStampMin = min(lMap.mTimeStampMin,lPoint.mTs)
		
		
	def run(self):
		lLoopCounter = 0
		lStart = time.time()
		# ask input provider for new data
		for (lSensorInputs,lTimestamp, lMoveLen, lMoveSpeed, lMoveAngle) in self.__mInputProvider:
			if lSensorInputs is None:
				break
			if len(lSensorInputs) != self.__mInputProvider.getSensorCount():
				continue
			
			lLoopCounter = lLoopCounter + 1
			lDelta = time.time() - lStart
			if lDelta > 2:
				#print('Sensor loop @' + str(lLoopCounter / lDelta) + ' loop/sec')
				lStart = time.time()
				lLoopCounter = 0	
			
			# update input filter with new values
			self.__mInputFilters.addSensorMeasures(lSensorInputs,lTimestamp)
			lFilteredSensorInputs = self.__mInputFilters.getFilteredResults()
			
			# how many distance do we travel 
			lDr = lMoveLen
			# with wich angle in radian
			lCmdAngle = math.radians(lMoveAngle)
			# compute turning circle radius 46cm is the length of imaginarium 
			lSinCmdAngle = math.sin(lCmdAngle)
			lTheta = 0.
			if lSinCmdAngle == 0.:
				lR = 0.
				lTheta = 0.
			else: 
				lR = 46. / lSinCmdAngle
				lTheta = lDr / lR

			lSinTheta = math.sin(lTheta)
			lCosTheta = math.cos(lTheta)
			# lXForm = np.matrix([lCosTheta, -lSinTheta, lDr * -lSinTheta],[lSinTheta, lCosTheta, lDr * lCosTheta],[0.,0.,1.])
			# a command angle of zero means that we are moving along y axis, so we need to map x axis of trigo circle to y axis of the robot
			# we apply a 90° rotation
			lXForm = np.matrix([[lCosTheta, -lSinTheta, lDr * -lSinTheta],[lSinTheta, lCosTheta, lDr * lCosTheta],[0.,0.,1.]])
			lXForm = None
			self.addMeasureToThreadMap(lSensorInputs,lFilteredSensorInputs,lTimestamp,lXForm,lMoveLen,lMoveSpeed)
			
			
			# update the map
			self.__mMutex.acquire()
			self.__mSharedMap = copy.deepcopy(self.__mThreadMap)
			self.__mMutex.release()

	


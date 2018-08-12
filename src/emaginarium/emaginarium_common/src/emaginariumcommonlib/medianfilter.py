from settings_store import settings_store_client
import collections
import math

class DataStreamFilter(settings_store_client.SettingsBase):

	def __init__(self, pSettingsPrefix):
		settings_store_client.SettingsBase.__init__(self)
		
		
			
		self.mWindowSize = 5
		self.mMinimumValidSamples = 2
		self.mLowPassFilterGain = 0.75
		
		self.__mValueQueues = []
		self.__mMedianValues = []
		self.__mFilteredValues = []
		self.__mTmpSorteBuffer = []
		
		self.registerAttributes([
			('mLowPassFilterGain',pSettingsPrefix + '/lowpassgain','gain of lowpass filter, 0 to disable'),
			('mMinimumValidSamples',pSettingsPrefix + '/minsamples','minimum valid samples'),
			('mWindowSize',pSettingsPrefix + '/windowsize','sample window size, < 0 to disable ')
			])

	def addValues(self, pValues):
		if self.mWindowSize <= 0:
			return pValues
		
		if len(pValues) != len(self.__mValueQueues):
			lSizeDelta = len(pValues) - len(self.__mValueQueues)
			if lSizeDelta < 0:
				self.__mValueQueues = self.__mValueQueues[:-lSizeDelta]
				self.__mMedianValues = self.__mMedianValues[:-lSizeDelta]
				self.__mFilteredValues = self.__mFilteredValues[:-lSizeDelta]
			else:
				for i in range(lSizeDelta):
					self.__mValueQueues.append(collections.deque())
					self.__mMedianValues.append(0.)
					self.__mFilteredValues.append(0.)

		for i in range(len(pValues)):
			lQueue = self.__mValueQueues[i];
			lQueue.append(pValues[i])
			while len(lQueue) > self.mWindowSize:
				lQueue.popleft()

			self.__mTmpSorteBuffer = []
			for v in lQueue:
				if v is None:
					continue
				self.__mTmpSorteBuffer.append(v)
			self.__mTmpSorteBuffer.sort()

			if len(self.__mTmpSorteBuffer) == 0 or len(self.__mTmpSorteBuffer) < self.mMinimumValidSamples:
				self.__mMedianValues[i] = 0.
				self.__mFilteredValues[i] = 0.
			else:
				self.__mMedianValues[i] = self.__mTmpSorteBuffer[int(math.floor(len(self.__mTmpSorteBuffer)/2))]
				if self.__mFilteredValues[i] is None or self.mLowPassFilterGain <= 0.:
					self.__mFilteredValues[i] = self.__mMedianValues[i]
				else:
					self.__mFilteredValues[i] = self.__mFilteredValues[i] + (self.__mMedianValues[i] - self.__mFilteredValues[i])*self.mLowPassFilterGain
		return self.__mFilteredValues

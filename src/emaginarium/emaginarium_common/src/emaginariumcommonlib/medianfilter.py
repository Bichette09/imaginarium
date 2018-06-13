from settings_store import settings_store_client

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
		if self.mWindowSize < 0:
			return pValues
		
		if len(pValues) != len(self.__mValueQueues):
			lSizeDelta = len(pValues) - len(self.__mValueQueues)
			if lSizeDelta < 0:
				self.__mValueQueues = self.__mValueQueues[:-lSizeDelta]
				self.mMedianValues = self.mMedianValues[:-lSizeDelta]
				self.mFilteredValues = self.mFilteredValues[:-lSizeDelta]
			else:
				for i in range(lSizeDelta):
					self.__mValueQueues.append(collections.deque())
					self.mMedianValues.append(None)
					self.mFilteredValues.append(None)

		for i in range(len(pValues)):
			lQueue = self.__mValueQueues[i];
			lQueue.append(pValues[i])
			while len(lQueue) > self.mWindowSize:
				lQueue.popleft()

			self.__mTmpSorteBuffer.clear()
			for v in lQueue:
				if v is None:
					continue
				self.__mTmpSorteBuffer.append(v)
			self.__mTmpSorteBuffer.sort()

			if len(self.__mTmpSorteBuffer) == 0 or len(self.__mTmpSorteBuffer) < self.mMinimumValidSamples:
				self.mMedianValues[i] = None
				self.mFilteredValues[i] = None
			else:
				self.mMedianValues[i] = self.__mTmpSorteBuffer[math.floor(len(self.__mTmpSorteBuffer)/2)]
				if self.mFilteredValues[i] is None or self.mLowPassFilterGain <= 0.:
					self.mFilteredValues[i] = self.mMedianValues[i]
				else:
					self.mFilteredValues[i] = self.mFilteredValues[i] + (self.mMedianValues[i] - self.mFilteredValues[i])*self.__mLowPassFilterGain
		return self.__mFilteredValues

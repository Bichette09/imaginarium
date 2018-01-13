# coding: utf8

import numpy as np
import tkinter as tk
import zmq
import json
import matplotlib.pyplot as plt

sSampleData = {
	'hitpoints' :[{
			'pt':[40,40],
			'dir':[1,1],
			'ts':1.2,
			'sensorid':1,
			'dist2sensor':10.,
			'influencedist':20
		}
	],
	'ts':0
}

sBackgroundColor = [238,238,238]#[238,238,238]
sMainGridColor = [190,190,190]
sSecGridColor = [210,210,210]

sSensorColors = [[0xbb,0x00,0xff], [0xff,0x00,0x62], [0xbf,0xff,0x00], [0xff,0x09,0x00], [0xff,0x00,0xae], [0x00,0xc8,0xff], [0x00,0x59,0xff], [0xff,0x7b,0x00], [0x0d,0xff,0x00], [0x00,0x95,0xff]]

def colorToString(pColor):
	return '#%02x%02x%02x' % (int(pColor[0]),int(pColor[1]),int(pColor[2]))

class PlotData(object):
	def __init__(self, pFormat):
		self.mFormat = pFormat
		self.mX = []
		self.mY = []
	
	def removeOldData(self, pMinX):
		while len(self.mX) > 0 and self.mX[0] < pMinX:
			self.mX.pop(0)
			self.mY.pop(0)
			
	def addData(self, pX, pY):
		if len(self.mX) > 0 and self.mX[-1] > pX:
			self.clear()
		self.mX.append(pX)
		self.mY.append(pY)
		
	
	def clear(self):
		self.mX = []
		self.mY = []

class PlotFigure(object):
	
	def __init__(self, pChannelFormat):
		# create objects to plot sensor values
		self.mSensorPlotFigure = plt.figure()
		self.mSensorPlotAxis = None
		self.mChannelData = {}
		for lChannel in pChannelFormat:
			self.mChannelData[lChannel] = PlotData(pChannelFormat[lChannel])
		
		self.clear()
		plt.ion()
		
	def show(self):
		self.mSensorPlotFigure.show()
	
	def clear(self):
		# clear the figure
		self.mSensorPlotFigure.clear()
		self.mSensorPlotAxis = self.mSensorPlotFigure.add_subplot(111)
		for lChannel in self.mChannelData:
			self.mChannelData[lChannel].clear()
			
	# add new data, they will be displayed after a call to updatePlot
	def addData(self, pChannel, pTs, pValue):
		if not pChannel in self.mChannelData:
			raise Exception('Undeclared channel')
		lData = self.mChannelData[pChannel].addData(pTs,pValue)
		
	# remove values older than pMinTs and redraw plot
	def updatePlot(self, pMinTs):
		self.mSensorPlotAxis.clear()
		for lChannel in self.mChannelData:
			lData = self.mChannelData[lChannel]
			lData.removeOldData(pMinTs)
			self.mSensorPlotAxis.plot(lData.mX,lData.mY,lData.mFormat)
		self.mSensorPlotAxis.set_ylim(ymin=0)
		
		
class MapViewer(object):

	# The class "constructor" - It's actually an initializer 
	def __init__(self):
		self.mNeedToRedrawGrid = True
		self.mNeedToRedrawMap = True
		self.mDataToPlot = None
		self.mCloseWindow = False
		
		self.mPlotFigure = PlotFigure({'raw':'r-','filtered':'b-'})
		
		# create zmq objects to receive data
		self.mZmqContext = zmq.Context()
		self.mZmqSocket = self.mZmqContext.socket(zmq.SUB)
		# self.mZmqSocket.connect ("tcp://10.0.0.109:5556")
		self.mZmqSocket.connect ("tcp://127.0.0.1:5556")
		self.mZmqSocket.setsockopt(zmq.SUBSCRIBE, b'mapviewer')

		self.mZmqPoller = zmq.Poller()
		self.mZmqPoller.register(self.mZmqSocket, zmq.POLLIN)
		
		
		# create gui
		self.mMainWindow =  tk.Tk()
		self.mMainWindow.protocol("WM_DELETE_WINDOW", self.on_closing)
	
		self.mCanvas = tk.Canvas(self.mMainWindow, width=600, height=600, background=colorToString(sBackgroundColor))
		self.mCanvas.pack(fill=tk.BOTH, expand=tk.YES)
		self.mCanvas.bind_all("<MouseWheel>", self.canvas_on_mousewheel)
		# self.mCanvas.bind_all("<Motion>", self.canvas_on_mousemove)
		# self.mCanvas.bind_all("<Leave>", self.canvas_on_mouseleave)
		self.mCanvas.bind('<Configure>', self.canvas_on_resize)
		
		
		lLabel = tk.Label(self.mMainWindow, text="Zoom factor")
		lLabel.pack(side=tk.LEFT,padx=5)
		
		self.mSpinBoxZoom = tk.Spinbox(self.mMainWindow,from_=1, to=200,  width=5, command=self.updateCanvas)
		self.mSpinBoxZoom.pack(side=tk.LEFT)
		# set default value
		self.mSpinBoxZoom.delete(0,"end")
		self.mSpinBoxZoom.insert(0,150)
		
		
		lLabel = tk.Label(self.mMainWindow, text="Graph sensor id")
		lLabel.pack(side=tk.LEFT,padx=5)
		self.mSensorIdSpinBox = tk.Spinbox(self.mMainWindow,from_=-2, to=9,  width=5, command=self.onGraphSensorIdChanged)
		self.mSensorIdSpinBox.pack(side=tk.LEFT)
		
		lLabel = tk.Label(self.mMainWindow, text="Graph sensor duration")
		lLabel.pack(side=tk.LEFT,padx=5)
		self.mSensorGraphDurationSpinBox = tk.Spinbox(self.mMainWindow,from_=2, to=60,  width=5)
		self.mSensorGraphDurationSpinBox.pack(side=tk.LEFT)
		# set default value
		self.mSensorGraphDurationSpinBox.delete(0,"end")
		self.mSensorGraphDurationSpinBox.insert(0,10)
		
		
	
	def on_closing(self):
		#~handle close manually to avoid dead lock with matplotlib figure
		self.mCloseWindow = True

	def onGraphSensorIdChanged(self):
		self.mPlotFigure.clear()
		self.mPlotFigure.show()
		
	
	def canvas_on_resize(self, event):
		self.mNeedToRedrawGrid = True
		
	# def canvas_on_mouseleave(self, event):
		# print(event)
		
	# def canvas_on_mousemove(self, event):
		# print(event)
		
	def canvas_on_mousewheel(self, event):
		lDelta = 5 
		if event.delta > 0:
			lDelta = -lDelta;
		lNewValue = max(min(200,float(self.mSpinBoxZoom.get()) + lDelta),10)
		
		# set default value
		self.mSpinBoxZoom.delete(0,"end")
		self.mSpinBoxZoom.insert(0,lNewValue)
		
		self.mNeedToRedrawGrid = True
		
		self.updateCanvas()
		
	def transformPoint(self, pPoint):
		lCanvasSize = np.array([int(self.mCanvas.winfo_width()),int(self.mCanvas.winfo_height())])
		lOffsetToCenter = lCanvasSize * 0.5
		lCmToPixelFactor = 100. / float(self.mSpinBoxZoom.get())
		lRes = pPoint * lCmToPixelFactor 
		lRes[1] = -lRes[1]
		return lRes + lOffsetToCenter
		
	def setData(self, pDataToPlot):
		# update the map
		self.mDataToPlot = pDataToPlot
		self.mNeedToRedrawMap = True
		self.updateCanvas()
		
		
		# update sensor input graph
		lSensorIdToMonitor = int(self.mSensorIdSpinBox.get())
		lSensorGraphDuration = int(self.mSensorGraphDurationSpinBox.get())
		if lSensorIdToMonitor == -1.:
			if 'speedgoal' in self.mDataToPlot:
				self.mPlotFigure.addData('raw',self.mDataToPlot['ts'],self.mDataToPlot['speedgoal'])
			if 'speedmeasured' in self.mDataToPlot :
				self.mPlotFigure.addData('filtered',self.mDataToPlot['ts'],self.mDataToPlot['speedmeasured'])
			self.mPlotFigure.updatePlot(self.mDataToPlot['ts'] - lSensorGraphDuration)
		
		elif lSensorIdToMonitor >= 0:
			if 'lastinput' in self.mDataToPlot and lSensorIdToMonitor < len(self.mDataToPlot['lastinput']):
				self.mPlotFigure.addData('raw',self.mDataToPlot['ts'],self.mDataToPlot['lastinput'][lSensorIdToMonitor])
			if 'lastfilteredinput' in self.mDataToPlot and lSensorIdToMonitor < len(self.mDataToPlot['lastfilteredinput']):
				self.mPlotFigure.addData('filtered',self.mDataToPlot['ts'],self.mDataToPlot['lastfilteredinput'][lSensorIdToMonitor])
			self.mPlotFigure.updatePlot(self.mDataToPlot['ts'] - lSensorGraphDuration)
		
		
	def updateCanvas(self):
		# draw grid
		if self.mNeedToRedrawGrid:
			self.mNeedToRedrawGrid = False
			#force redraw of map
			self.mNeedToRedrawMap = True
			# remove previous grid
			self.mCanvas.delete('grid')
			
			lMainGridColorString = colorToString(sMainGridColor)
			lSecGridColorString = colorToString(sSecGridColor)
			for idx in range(-1000,1000,20):
				lA = self.transformPoint(np.array([idx,-1000]))
				lB = self.transformPoint(np.array([idx,1000]))
				lC = self.transformPoint(np.array([-1000,idx]))
				lD = self.transformPoint(np.array([1000,idx]))
				
				if (idx % 100) == 0:
					self.mCanvas.create_line(lA[0],lA[1],lB[0],lB[1],dash=(5,),tags='grid', fill=lMainGridColorString)
					self.mCanvas.create_line(lC[0],lC[1],lD[0],lD[1],dash=(5,),tags='grid', fill=lMainGridColorString)
				else:
					self.mCanvas.create_line(lA[0],lA[1],lB[0],lB[1],dash=(1,),tags='grid', fill=lSecGridColorString)
					self.mCanvas.create_line(lC[0],lC[1],lD[0],lD[1],dash=(1,),tags='grid', fill=lSecGridColorString)
		
		# draw center
		if self.mNeedToRedrawMap:
			self.mNeedToRedrawMap = False
			self.mCanvas.delete('robot')
			lA = self.transformPoint(np.array([-5,-5]))
			lB = self.transformPoint(np.array([5,5]))
			self.mCanvas.create_oval(lA[0], lA[1], lB[0], lB[1], tags='robot')
			
			# draw sensor location
			self.mCanvas.delete('sensorloc')
			if self.mDataToPlot is not None and 'sensorloc' in self.mDataToPlot:
				for i in range(0,len(self.mDataToPlot['sensorloc'])):
					lPoint = np.array(self.mDataToPlot['sensorloc'][i])
					lA = self.transformPoint(lPoint)
					lColor = sSensorColors[i]
					lColorString = '#%02x%02x%02x' % (int(lColor[0]),int(lColor[1]),int(lColor[2]))
					self.mCanvas.create_text(lA[0],lA[1],text=str(i),tags='sensorloc', fill=lColorString)
			# draw points
			self.mCanvas.delete('hitpoint')
			if self.mDataToPlot is not None and 'hitpoints' in self.mDataToPlot:
				lNbMeasures = len(self.mDataToPlot['hitpoints'])
				
				
				lBaseColor = np.array([0,0,255])
				lForegroundColor = np.array([238,238,238])
				# lMixFactor = 0.1
				# lMixStep = (1 - lMixFactor) / lNbMeasures
				for lEntry in reversed(self.mDataToPlot['hitpoints']):
					# compute color
					# lMixFactor = lMixFactor + lMixStep
					# lColor = (lBaseColor * lMixFactor + lForegroundColor * (1. - lMixFactor)).clip(0,255)
					# lColor = lBaseColor
					lPoint = np.array(lEntry['pt'])
					lDir = np.array(lEntry['dir'])
					lTs = lEntry['ts']
					lSensorId = lEntry['sensorid']
					lHalfInfluenceLength = lEntry['influencedist']*0.5
					lColorIdx = lSensorId % len(sSensorColors)
					lColor = sSensorColors[lColorIdx]
					lColorString = '#%02x%02x%02x' % (int(lColor[0]),int(lColor[1]),int(lColor[2]))
					
					
					if np.linalg.norm(lDir) > 0:
						lDir = lDir / np.linalg.norm(lDir)
					lA = self.transformPoint(lPoint) + np.array([-2,-2])
					lB = self.transformPoint(lPoint) + np.array([2,2])
					self.mCanvas.create_oval(lA[0], lA[1], lB[0], lB[1], tags='hitpoint', fill=lColorString, outline=lColorString)
					
					lDirCross = np.array([lDir[1],-lDir[0]])
					lA = self.transformPoint(lPoint + lDirCross * lHalfInfluenceLength)
					lB = self.transformPoint(lPoint + lDirCross * -lHalfInfluenceLength)
					self.mCanvas.create_line(lA[0],lA[1],lB[0],lB[1],tags='hitpoint', fill=lColorString)
		
	def onLoop(self):
		if self.mCloseWindow:
			self.mMainWindow.destroy()
			del self.mMainWindow
			plt.close('all')
			
		else:
			self.mMainWindow.after(15, self.onLoop)
			
			lDataToPlot = None
			while True:
				socks = dict(self.mZmqPoller.poll(0))
				if socks:
					if socks.get(self.mZmqSocket) == zmq.POLLIN:
						lReceivedData = self.mZmqSocket.recv(zmq.NOBLOCK)
						print(lReceivedData[9:].decode('utf-8'))
						lDataToPlot = json.loads(lReceivedData[9:].decode('utf-8'))
					else:
						break
				else:
					break
			
			if lDataToPlot is not None:
				self.setData(lDataToPlot)
			
			self.updateCanvas()

	def runMainLoop(self):
		self.mMainWindow.after(500, self.onLoop)
		self.mMainWindow.mainloop()
		

if __name__ == '__main__':
	
	lViewer = MapViewer()
	lViewer.setData(sSampleData)
	lViewer.runMainLoop()
	del lViewer

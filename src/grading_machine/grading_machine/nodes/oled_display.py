#!/usr/bin/env python
# coding: utf-8

from settings_store import settings_store_client
# from luma.core.render import canvas
from luma.core import cmdline, error
from luma.core.render import canvas
from PIL import Image, ImageDraw, ImageFont

import grading_machine.msg
import rospy
import textwrap
import math

class OledDisplaySettings(settings_store_client.SettingsBase):
	
	def __init__(self):
		settings_store_client.SettingsBase.__init__(self)
		self.mBrightness = 5
		
		self.registerAttributes([
			('mBrightness','oled_display/mBrightness',1,5,'brightness of display [1-5]'),
			])

class Display():
	def __init__(self):
		
		parser = cmdline.create_parser(description='luma.examples arguments')
		args = parser.parse_args([])
		self.__mStateDeclarator = settings_store_client.StateDeclarator()
		
		# fc-list | grep Vera
		self.__mFontName = {}
		self.__mFontName[(True,True)] = "VeraBI.ttf"
		self.__mFontName[(True,False)] = "VeraBd.ttf"
		self.__mFontName[(False,True)] = "VeraIt.ttf"
		self.__mFontName[(False,False)] = "Vera.ttf"
		
		self.__mDevice = None
		
		# create device
		try:
			self.__mDevice = cmdline.create_device(args)
			self.__mDevice.clear()
			
			#create buffer
			self.__mBuffer = Image.new("RGB", self.__mDevice.size)
			self.__mBufferTmp = Image.new("RGB", self.__mDevice.size)
			
			self.__mHasPendingChanges = True
			self.__mPreviousBrightness = -1
			
			rospy.Subscriber('/oled_display/draw_rect',grading_machine.msg.DisplayDrawRect,self.__onDrawRect)
			rospy.Subscriber('/oled_display/draw_text',grading_machine.msg.DisplayDrawText,self.__onDrawText)
			self.__mStateDeclarator.setState('i2c_display','ok')
		except:
			self.__mDevice = None
			rospy.logerr('Fail to initialize i2c oled display')
			self.__mStateDeclarator.setState('i2c_display','error')
		
		
	
	def setBrightness(self, pVal):
		if (self.__mDevice is None):
			return
		lVal = 1 + max(0,(pVal - 1) / 4.)*254
		lVal = int(lVal)
		if lVal == self.__mPreviousBrightness:
			return
		self.__mPreviousBrightness = lVal
		self.__mDevice.contrast(lVal)
	
	def drawText(self, pX,pY,pMaxWidth,pMaxHeight, pText, pBold, pItalic, pSize):
		lDraw = ImageDraw.Draw(self.__mBufferTmp)
		lFontSize = pSize if pSize > 0 else 10
		lFont = ImageFont.truetype(self.__mFontName[(pBold,pItalic)], lFontSize)
		lWidth = pMaxWidth if pMaxWidth > 0 else self.__mBuffer.size[0]
		lHeight = pMaxHeight if pMaxHeight > 0 else self.__mBuffer.size[1]
		lDraw.rectangle([0, 0, lWidth,lHeight],fill='black')
		# draw text
		lCharWidth = lDraw.textsize(pText)[0] / float(len(pText))
		lWrapSize = int(math.floor(lWidth/lCharWidth))
		lYOffset = 0
		for line in textwrap.wrap(pText, width=lWrapSize):
			lDraw.text((0,lYOffset),line, fill='white', font=lFont)
			lYOffset += lDraw.textsize(line)[1]
		
		# copy target area to output buffer
		self.__mBuffer.paste(self.__mBufferTmp.crop(box = (0,0,lWidth,lHeight)),[pX, pY])
		self.__mHasPendingChanges = True
	
	def drawRectangle(self,pX,pY,pWidth,pHeight,pFill,pOutline):
		lDraw = ImageDraw.Draw(self.__mBuffer)
		lWidth = pWidth if pWidth > 0 else self.__mBuffer.size[0]
		lHeight = pHeight if pHeight > 0 else self.__mBuffer.size[1]
		lDraw.rectangle([pX, pY, pX + lWidth, pY + lHeight],fill= 'white' if pFill else 'black',outline= 'white' if pOutline else 'black')
		self.__mHasPendingChanges = True
	
	def clear(self):
		self.drawRectangle(0,0,self.__mBuffer.size[0],self.__mBuffer.size[1],False,False)
		
	def swapDisplay(self):
		if (self.__mDevice is None) or (not self.__mHasPendingChanges):
			return
		self.__mHasPendingChanges = False
		lCanvas = canvas(self.__mDevice)
		with lCanvas as draw:
			lCanvas.image.paste(self.__mBuffer)
			
	def __onDrawRect(self,pParam):
		self.drawRectangle(pParam.x,pParam.y,pParam.width,pParam.height,pParam.fill,pParam.outline)
	
	def __onDrawText(self,pParam):
		self.drawText(pParam.x,pParam.y,pParam.width,pParam.height,pParam.text,pParam.bold,pParam.italic, pParam.fontsize)
		
if __name__ == "__main__":
	rospy.init_node('oled_display')
	
	lSettings = OledDisplaySettings()
	
	# lSubScriber = rospy.Publisher('/oled_display/clear_area',grading_machine.msg.DisplayClearArea, queue_size = 1)
	# lSubScriber = rospy.Publisher('/oled_display/draw_text',grading_machine.msg.DisplayClearArea, queue_size = 1)
	
	lDisplay = Display()
	
	#sDisplay.drawText(0,0,128,64,'Hello World !',False,False,10)
	# sDisplay.drawText(0,58,32,24,'Hello World !',False,False,10)

	while not rospy.core.is_shutdown():
		rospy.rostime.wallsleep(0.1)
		lDisplay.swapDisplay()
		lDisplay.setBrightness(lSettings.mBrightness)
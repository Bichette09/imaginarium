#!/usr/bin/env python
# coding: utf-8

import rospy
import grading_machine.msg


## base class that should be derived to declare/synchronize settings with settings_store server
# 
class Display:

	def __init__(self):
		self.mRectPub = rospy.Publisher('/oled_display/draw_rect', grading_machine.msg.DisplayDrawRect, queue_size=1)
		self.mTextPub = rospy.Publisher('/oled_display/draw_text', grading_machine.msg.DisplayDrawText, queue_size=1)
		# add a sleep to wait for ros init
		rospy.rostime.wallsleep(0.2)
	def drawRect(self, x, y, w = 0, h = 0, fill = False, outline = True):
		self.mRectPub.publish(grading_machine.msg.DisplayDrawRect(x,y,w,h,fill,outline))

class TextArea:
	def __init__(self, pDisplay, area = [0,0,0,0]):
		self.mDisplay = pDisplay
		self.area = area
		self.bold = None
		self.italic = None
		self.size = None
		self.text = None
	
	def clear():
		self.mDisplay.drawRect(area[0],area[1],area[2],area[3],False,False)
	
	def drawText(self, text, size=10, bold = False, italic = False,force = False):
		if (self.text != text or
				self.size != size or
				self.bold != bold or
				self.italic != italic or
				force):
			self.text = text
			self.size = size
			self.bold = bold
			self.italic = italic
			self.mDisplay.mTextPub.publish(grading_machine.msg.DisplayDrawText(text,self.area[0],self.area[1],self.area[2],self.area[3],size,bold,italic))
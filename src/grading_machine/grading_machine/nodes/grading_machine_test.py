#!/usr/bin/env python
# coding: utf-8

# import grading_machine.srv
# import grading_machine.msg
from settings_store import settings_store_client
import rospy
import sys
from grading_machine import oled_display_client

class GradingSettings(settings_store_client.SettingsBase):
	
	def __init__(self):
		settings_store_client.SettingsBase.__init__(self)
		# self.mUpdatePeriod = 1.
		self.mMessage = 'yopyop'
		
		self.registerAttributes([
			# ('mUpdatePeriod','GradingSettings/UpdatePeriod',0.01,10),
			('mMessage','GradingSettings/Message'),
			])
		
	
if __name__ == "__main__":
	rospy.init_node('grading_machine_test')
	
	lSettings = GradingSettings()

	lDisp = oled_display_client.Display()
	rospy.logwarn('Test ready')
	lDisp.drawRect(0,0,15,15)
	lTextArea = oled_display_client.TextArea(lDisp,[0,0,128,96])
	
	while not rospy.core.is_shutdown():
		rospy.rostime.wallsleep(0.1)
		lTextArea.drawText(lSettings.mMessage)
		
	
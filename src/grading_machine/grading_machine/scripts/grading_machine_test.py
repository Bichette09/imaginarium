#!/usr/bin/env python
# coding: utf-8

# import grading_machine.srv
# import grading_machine.msg
from settings_store import settings_store_client
import rospy
import sys
import codecs
import json
import os

class GradingSettings(settings_store_client.SettingsBase):
	
	def __init__(self):
		settings_store_client.SettingsBase.__init__(self)
		self.mUpdatePeriod = 1.
		self.mMessage = 'yopyop'
		
		self.registerAttributes([
			('mUpdatePeriod','GradingSettings/UpdatePeriod'),
			('mMessage','GradingSettings/Message'),
			])
		


if __name__ == "__main__":
	rospy.init_node('grading_machine_test')
	
	
	lSettings = GradingSettings()
	
	while not rospy.core.is_shutdown():
		rospy.rostime.wallsleep(lSettings.mUpdatePeriod)
		rospy.logwarn(lSettings.mMessage)
		pass
	
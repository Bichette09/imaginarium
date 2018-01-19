#!/usr/bin/env python
# coding: utf-8

import rospy
import settings_store.msg
import settings_store.srv

class SettingsBase:

	def __init__(self):
		rospy.Subscriber('settings_store/change',settings_store.msg.change,self)
		
	# def __call__(self, params):
		

	
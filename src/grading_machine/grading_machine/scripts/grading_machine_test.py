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



if __name__ == "__main__":
	rospy.init_node('grading_machine_test')
	
	
	lSettings = settings_store_client.SettingsBase()
	
	while not rospy.core.is_shutdown():
		rospy.rostime.wallsleep(1.)
		rospy.logwarn('coucou')
	
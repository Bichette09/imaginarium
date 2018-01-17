#!/usr/bin/env python
# coding: utf-8

import template_python.srv
import template_python.msg
import rospy
import sys
import codecs
import json
import os



if __name__ == "__main__":
	rospy.init_node('grading_machine_test')
	
	while not rospy.core.is_shutdown():
		rospy.rostime.wallsleep(1.)
		rospy.logwarn('coucou')
	
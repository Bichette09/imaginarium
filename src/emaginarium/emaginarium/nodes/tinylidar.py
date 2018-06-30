#!/usr/bin/env python
# coding: utf-8

import emaginarium.msg
import emaginariumcommonlib
import emaginariumlib
import rospy
import os
import smbus
import serial
import numpy as np
import math
import time

if __name__ == "__main__":
	
	os.getcwd()
	rospy.init_node('tinylidar')

	
	sRosPublisher = rospy.Publisher('emaginarium/TinyLidar', emaginarium.msg.TinyLidar, queue_size=5)
	lReader = emaginariumlib.ArduinoFloatArrayReader(rospy.get_param('/tinylidar/serialPort'),1,'tinylidar')
	
	while not rospy.core.is_shutdown():
	
		lRawDistance = lReader.readDistance()
		if lRawDistance is None:
			continue
		
		# Message publication
		sRosPublisher.publish(emaginarium.msg.TinyLidar(lRawDistance))

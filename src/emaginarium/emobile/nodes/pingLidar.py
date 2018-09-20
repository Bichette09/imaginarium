#!/usr/bin/env python
# coding: utf-8


import emobile.msg
import emaginarium_common.msg
import std_msgs
import rospy
import os
from settings_store import settings_store_client
import time



rospy.init_node('pingLidar')

lStateDeclarator = settings_store_client.StateDeclarator()

tof = None
try:
	import VL53L1X
	tof = VL53L1X.VL53L1X(i2c_bus=1, i2c_address=0x29)
	tof.open() # Initialise the i2c bus and configure the sensor
	lStateDeclarator.setState('init/VL53L1X',True)
except:
	rospy.logerr('Fail to init VL53L1X')
	lStateDeclarator.setState('init/VL53L1X',False)
	
sRosPublisherPingLidardDist = rospy.Publisher('emobile/PingLidarDist', std_msgs.msg.Float32, queue_size=1)
lRate = rospy.Rate(42)

while not rospy.is_shutdown():
	if tof is not None:
		tof.start_ranging(1) # Start ranging, 1 = Short Range, 2 = Medium Range, 3 = Long Range
		distance_in_m = tof.get_distance()/1000. # Grab the range in mm and convert in meters
		tof.stop_ranging() # Stop ranging
		sRosPublisherPingLidardDist.publish(std_msgs.msg.Float32(distance_in_m))
	
	lRate.sleep()
